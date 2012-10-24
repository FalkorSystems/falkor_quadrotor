#!/usr/bin/env python
import roslib
roslib.load_manifest( 'falkor_quadrotor_sensors' )
import rospy

from falkor_quadrotor_sensors.srv import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
import numpy as np
import tf

class CalibrationCollection:
    def __init__(self, time):
        self.time = time
        self.collecting = False

        self.init_data()
        self.mag_sub = rospy.Subscriber( 'magnetic',
                                         Vector3Stamped,
                                         self.mag_cb )


        self.world_frame = rospy.get_param( '~world_frame', "/nav" )
        self.magnetic_frame = rospy.get_param( '~magnetic_field_frame', self.world_frame + "/magnet" )
        self.tf_prefix = rospy.get_param( '~tf_prefix', '' )
        self.imu_sub = rospy.Subscriber( 'imu/data',
                                         Imu,
                                         self.imu_cb )

    def init_data( self ):
        self.mag_vectors = None
        self.imu_angular_velocity = None
        self.imu_orientation = None
        self.imu_linear_acceleration = None


    def mag_cb( self, data ):
        if self.collecting == False:
            return

        mag_vector = np.array( [ [ data.vector.x,
                                   data.vector.y,
                                   data.vector.z ] ] )
        if self.mag_vectors == None:
            self.mag_vectors = mag_vector
        else:
            self.mag_vectors = np.append( self.mag_vectors, mag_vector, axis=0 )
    
    def imu_cb( self, data ):
        if self.collecting == False:
            return

        angular_velocity = np.array( [ [ data.angular_velocity.x,
                                         data.angular_velocity.y,
                                         data.angular_velocity.z ] ] )
        linear_acceleration = np.array( [ [ data.linear_acceleration.x,
                                            data.linear_acceleration.y,
                                            data.linear_acceleration.z ] ] )
        orientation = np.array( [ [ data.orientation.x,
                                    data.orientation.y,
                                    data.orientation.z,
                                    data.orientation.w ] ] )

        if self.imu_angular_velocity == None:
            self.imu_angular_velocity = angular_velocity
            self.imu_linear_acceleration = linear_acceleration
            self.imu_orientation = orientation
        else:
            self.imu_angular_velocity = np.append( self.imu_angular_velocity,
                                                   angular_velocity, axis = 0 )
            self.imu_linear_acceleration = np.append( self.imu_linear_acceleration,
                                                      linear_acceleration, axis = 0 )
            self.imu_orientation = np.append( self.imu_orientation,
                                              orientation, axis = 0 )

    def calculate( self ):
        # average and normalize!
        starting_imu_orientation = tf.transformations.unit_vector( np.mean( self.imu_orientation, axis = 0 ) )
        starting_mag_vector = tf.transformations.unit_vector( np.mean( self.mag_vectors, axis = 0 ) )

        # rotate the mag_vector by the imu orientation,
        # now we have mag_vector with respect to IMU 0,0,0,1
        rotated_mag_vector = tf.transformations.quaternion_matrix( starting_imu_orientation )[:3,:3].dot( starting_mag_vector )
        mag_yaw = np.arctan2( -rotated_mag_vector[1], rotated_mag_vector[0] )
        self.mag_quaternion = tf.transformations.quaternion_from_euler( 0, 0, mag_yaw )

        # rotate gravity vector by the imu orientation
        # now we have gravity with respect to IMU 0,0,0,1
        gravity_vector = - np.mean( self.imu_linear_acceleration, axis = 0 ) 
        gravity_vector_normalized = tf.transformations.unit_vector( gravity_vector )
        rotated_gravity_vector = tf.transformations.quaternion_matrix( starting_imu_orientation )[:3,:3].dot( gravity_vector_normalized )
        gravity_pitch = np.arctan2( rotated_gravity_vector[0], -rotated_gravity_vector[2] )
        gravity_roll = np.arctan2( rotated_gravity_vector[1], -rotated_gravity_vector[2] )

        self.gravity_quaternion = tf.transformations.quaternion_from_euler( gravity_roll, gravity_pitch, 0.0 )
        self.gravity_magnitude = np.linalg.norm( gravity_vector )
        
    def publish( self ):
        broadcaster = tf.TransformBroadcaster()
        broadcaster.sendTransform( (0.0, 0.0, 0.0),
                                   self.mag_quaternion,
                                   rospy.Time.now(),
                                   self.tf_prefix + "/imu_base_mag",
                                   self.magnetic_frame )

        broadcaster.sendTransform( (0.0, 0.0, 0.0),
                                   self.gravity_quaternion,
                                   rospy.Time.now(),
                                   self.tf_prefix + "/imu_base",
                                   self.tf_prefix + "/imu_base_mag" )

    def collect( self ):
        self.collecting = True
        rospy.sleep( rospy.Duration( self.time ) )
        self.collecting = False

class CalibrateSensors:
    def __init__(self):
        self.calibrated = False
        self.calibrator = None
        self.calibrating = False

        self.service = rospy.Service( 'calibrate_sensors',
                                      Calibrate,
                                      self.calibrate )
        self.service_done = rospy.Service( 'calibrate_done',
                                           CalibrateDone,
                                           self.calibrate_done )


    def calibrate_done( self, req ):
        return { 'done': self.calibrated }

    def calibrate( self, req ):
        # if a calibration already happening, return failure
        if self.calibrating:
            return { 'success': False }

        self.calibrating = True
        self.calibrated = False
        # setup data collection
        self.calibrator = CalibrationCollection( req.time )
        
        # collect the data
        self.calibrator.collect()

        # Now that we've collected the data, calculate base values
        self.calibrator.calculate()

        # reset stored data
        self.calibrator.init_data()
        self.calibrated = True
        self.calibrating = False
        return { 'success': True }

    def run( self ):
        r = rospy.Rate( 10 )

        while not rospy.is_shutdown():
            if self.calibrated:
                self.calibrator.publish()
            r.sleep()

if __name__ == '__main__':
    rospy.init_node( 'calibrate_sensors' )
    server = CalibrateSensors()
    server.run()
