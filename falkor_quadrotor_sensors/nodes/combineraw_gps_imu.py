#!/usr/bin/env python
import roslib
roslib.load_manifest( 'falkor_quadrotor_sensors' )

import rospy
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from nav_msgs.msg import *
import tf
import numpy as np

class FalkorCombineGpsImu:
    def __init__( self ):
        self.gps_topic = rospy.get_param( "~gps_fix_topic", "fix_m/state" )
        self.imu_topic = rospy.get_param( "~imu_topic", "imu" )
        self.world_frame = rospy.get_param( "~world_frame", "/nav" )
        self.gravity = rospy.get_param( "~gravity", 9.82 )

        self.pose_pub = rospy.Publisher( "raw/pose", PoseWithCovarianceStamped )
        self.state_pub = rospy.Publisher( "raw/state", Odometry )
        self.twist_pub = rospy.Publisher( "raw/twist", TwistWithCovarianceStamped )
        self.accel_pub = rospy.Publisher( "raw/accel", Vector3Stamped )

        self.listener = tf.TransformListener()

        self.gps_sub = rospy.Subscriber( self.gps_topic, Odometry, self.gps_cb )
        self.imu_sub = rospy.Subscriber( self.imu_topic, Imu, self.imu_cb )

        self.seq = 0

        self.pose = PoseWithCovarianceStamped()
        self.pose.pose.pose.orientation = Quaternion( 0, 0, 0, 1 )
        self.accel = Vector3Stamped()
        self.twist = TwistWithCovarianceStamped()
        self.state = Odometry()

        self.rate = rospy.Rate( rospy.get_param( "~update_rate", 10 ) )

    def gps_cb( self, data ):
        # The gps pose is in the world frame, but double check
        if not data.header.frame_id == self.world_frame:
            raise Exception( "gps data frame does not match world frame" )

        self.pose.pose.pose.position = data.pose.pose.position

        # Set the pose portions of the covariance
        data_covariance = np.array( data.pose.covariance )
        gps_pose_covariance = data_covariance.reshape((6,6))[:3,:3].reshape((3,3))
        my_pose_covariance_w_orient = np.array( self.pose.pose.covariance ).reshape( ( 6, 6 ) )
        my_pose_covariance_w_orient[:3,:3] = gps_pose_covariance

        self.pose.pose.covariance = list( my_pose_covariance_w_orient.reshape( 36 ) )

        # now do twist
        self.twist.twist.twist.linear = data.twist.twist.linear

        # set the twist covariance
        data_covariance = np.array( data.twist.covariance )
        gps_twist_covariance = data_covariance.reshape( ( 6,6 ) )[:3,:3].reshape((3,3))
        my_twist_covariance_w_angular = np.array( self.twist.twist.covariance ).reshape( ( 6, 6 ) )
        my_twist_covariance_w_angular[:3,:3] = gps_twist_covariance

        self.twist.twist.covariance = list( my_twist_covariance_w_angular.reshape( 36 ) )

    def imu_cb( self, data ):
        # wait for transform
        try:
            self.listener.waitForTransform( data.header.frame_id,
                                            self.world_frame,
                                            data.header.stamp,
                                            rospy.Duration( 4.0 ) )
            quaternion_imu = QuaternionStamped( data.header, data.orientation )
            angular_velocity_imu = Vector3Stamped( data.header, data.angular_velocity )
            accel_imu = Vector3Stamped( data.header, data.linear_acceleration )

            # convert from the imu frame to the world frame
            self.pose.pose.pose.orientation = self.listener.transformQuaternion( self.world_frame, quaternion_imu ).quaternion
            self.twist.twist.twist.angular = self.listener.transformVector3( self.world_frame, angular_velocity_imu ).vector
            self.accel.vector = self.listener.transformVector3( self.world_frame, accel_imu ).vector

            # we should throw in a covariance for the orientation and angular twist here but I don't know how

            # remove gravity
            self.accel.vector.z -= self.gravity

        except (tf.LookupException, tf.Exception,
                tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn( "combineraw_gps_imu: transform exception: %s", str( e ) )

    def publish( self ):
        self.seq += 1
        self.accel.header = self.twist.header = self.pose.header = self.state.header = Header( self.seq, rospy.Time.now(), self.world_frame )
        self.pose_pub.publish( self.pose )
        self.twist_pub.publish( self.twist )

        self.state.twist = self.twist.twist
        self.state.pose = self.pose.pose

        self.state_pub.publish( self.state )
        self.accel_pub.publish( self.accel )

    def run( self ):
        while not rospy.is_shutdown():
            self.publish()
            self.rate.sleep()


def main():
    rospy.init_node('combineraw_gps_imu')
    combiner = FalkorCombineGpsImu()
    try:
        combiner.run()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__  == '__main__':
    main()

            
                                    
            

        
        