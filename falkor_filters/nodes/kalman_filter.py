#!/usr/bin/env python
import roslib; roslib.load_manifest( 'falkor_filters' )
import rospy

from nav_msgs.msg import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from falkor_msgs.msg import *

import numpy as np
import tf

class KalmanFilter:
    def __init__( self ):
        # Starting out we have no idea what the deal is
        self.prior_state = np.asmatrix( np.zeros( ( 9, 1 ) ) )
        self.prior_cov = np.asmatrix( np.identity( 9 ) * 100 )
    
        self.posterior_state = self.prior_state
        self.posterior_cov = self.prior_cov

        # the variance around position and velocity should be fairly low 
        # because those are pretty deterministic
        # it should be higher around acceleration
        # ( pos stddev = 10cm, vel stddev = 10cm/s, accel stddev = ( 10, 10, 10 ) m/s2)
        self.transition_cov = np.asmatrix( np.diag( np.append( np.ones( 6 ) * 0.01, np.ones( 3 ) * 5 ) ) )

    # Build transition matrix for a time interval dt
    def F( self, dt ):
        id_6 = np.identity( 6 )
        dt_6 = id_6 * dt
        zeros_63 = np.zeros( ( 6, 3 ) )
        zeros_36 = np.zeros( ( 3, 6 ) )

        return np.asmatrix( np.vstack( ( np.hstack( ( id_6, zeros_63 ) ) + np.hstack( ( zeros_63, dt_6 ) ),
                                         np.hstack( ( zeros_36, np.identity( 3 ) ) ) ) ) )

    # measurement matrix for gps
    def H_gps( self ):
        id_6 = np.identity( 6 )
        zeros_63 = np.zeros( ( 6, 3 ) )
        return np.asmatrix( np.hstack( ( id_6, zeros_63 ) ) )

    # measurement matrix for baro & sonar
    def H_zdot( self ):
        return np.asmatrix( np.hstack( ( np.zeros( ( 1, 5 ) ),
                                         np.ones( ( 1, 1 ) ),
                                         np.zeros( ( 1, 3 ) ) ) ) )

    # measurement matrix for imu
    def H_imu( self ):
        id_3 = np.identity( 3 )
        zeros_36 = np.zeros( ( 3, 6 ) )
        return np.asmatrix( np.hstack( ( zeros_36, id_3 ) ) )

    # generic prediction step
    def predict( self, dt ):
        F = self.F( dt )
        try:
            self.prior_state = F * self.posterior_state 
            self.prior_cov = F * self.posterior_cov * F.T + self.transition_cov
        except ValueError as e:
            rospy.logwarn( "Exception in matrix multiply %s x %s" % (
                    str( F ), str( self.posterior_state ) ) )

    def I( self ):
        return np.asmatrix( np.identity( 9 ) )

    # generic update step
    def update( self, measurement, measurement_cov, H ):
        innovation = measurement - H * self.prior_state
        innovation_cov = H * self.prior_cov * H.T + measurement_cov
        kalman_gain = self.prior_cov * H.T * np.linalg.inv( innovation_cov )
        self.posterior_state = self.prior_state + kalman_gain * innovation
        if self.posterior_state.shape != ( 9, 1 ):
            import pdb; pdb.set_trace()

        self.posterior_cov = ( self.I() - kalman_gain * H ) * self.prior_cov

        return( self.posterior_state, self.posterior_cov )

    def update_zdot( self, zdot, zdot_cov, dt ):
        self.predict( dt )
        res = self.update( zdot, zdot_cov, self.H_zdot() )
        return res

    def update_gps( self, gps, gps_cov, dt ):
        self.predict( dt )
        res = self.update( gps, gps_cov, self.H_gps() )
        return res

    def update_imu( self, imu, imu_cov, dt ):
        self.predict( dt )
        res = self.update( imu, imu_cov, self.H_imu() )
        return res

class EmaFilter:
    def __init__(self,alpha):
        self.alpha = alpha
        self.value = None

    def update(self,new_value):
        if self.value == None:
            self.value = new_value
            return self.value
        else:
            self.value = self.alpha*new_value + (1-self.alpha)*self.value
            return self.value

class Filter:
    def __init__(self):
        self.gps_topic = rospy.get_param( "~gps_topic", "raw/state" )
        self.imu_topic = rospy.get_param( "~imu_topic", "raw/accel" )
        self.baro_topic = rospy.get_param( "~baro_topic", "altimeter" )
        self.sonar_topic = rospy.get_param( "~sonar_topic", "sonar_height" )

        self.pose_pub = rospy.Publisher( "pose", PoseWithCovarianceStamped )
        self.state_pub = rospy.Publisher( "state", Odometry )
        self.twist_pub = rospy.Publisher( "twist", TwistWithCovarianceStamped )
        self.accel_pub = rospy.Publisher( "accel", Vector3Stamped )

        self.tf_prefix = rospy.get_param( "~tf_prefix", "" )

        self.world_frame = rospy.get_param( "~world_frame", "/map" )

        self.last_update = None
        self.last_sonar_time = None
        self.last_baro_time = None
        self.last_sonar = None
        self.last_baro = None

        self.sonar_filter = EmaFilter(0.9)
        self.baro_filter = EmaFilter(0.1)

        self.filter = KalmanFilter()
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

        # TODO: Configure proper covariances
        self.sonar_zdot_cov = np.matrix( 0.1 )
        self.baro_zdot_cov = np.matrix( 5 )
        self.imu_cov = np.asmatrix( np.identity( 3 ) ) * 0.1
        self.gps_data = None

        # TODO: wrap these four callbacks into a mutex
        self.gps_sub = rospy.Subscriber( self.gps_topic, Odometry, self.gps_cb, queue_size=1 )
        self.imu_sub = rospy.Subscriber( self.imu_topic, Vector3Stamped, self.imu_cb, queue_size=1 )
        self.sonar_sub = rospy.Subscriber( self.sonar_topic, Range, self.sonar_cb, queue_size=1 )
        self.baro_sub = rospy.Subscriber( self.baro_topic, Altimeter, self.baro_cb, queue_size=1 )

    def get_dt( self, now ):
        # ignore the stamp
        now = rospy.Time.now()
        if self.last_update == None:
            self.last_update = now
            return 0

        dt = ( now - self.last_update ).to_sec()
        if dt >= 0:
            self.last_update = now

        return dt

    def gps_cb( self, data ):
#        print "gps stamp: %10.4f/%10.4f" % ( data.header.stamp.to_sec(), rospy.Time.now().to_sec() )

        self.gps_data = data
        dt = self.get_dt( data.header.stamp )
        if dt < 0:
            return
        
        gps_data = np.matrix( [ data.pose.pose.position.x, 
                                data.pose.pose.position.y,
                                data.pose.pose.position.z,
                                data.twist.twist.linear.x,
                                data.twist.twist.linear.y,
                                data.twist.twist.linear.z ] ).T

        gps_pos_cov = np.matrix( data.pose.covariance ).reshape( ( 6, 6 ) )[:3,:3]
        gps_vel_cov = np.matrix( data.twist.covariance ).reshape( ( 6, 6 ) )[:3,:3]

        zeros_33 = np.asmatrix( np.zeros( ( 3, 3 ) ) )
        gps_cov = np.vstack( ( np.hstack( ( gps_pos_cov, zeros_33 ) ),
                               np.hstack( ( zeros_33, gps_vel_cov ) ) ) )

        res = self.filter.update_gps( gps_data, gps_cov, dt )
        self.publish_state( res, rospy.Time.now() )

    def imu_cb( self, data ):
#        print "imu stamp: %10.4f/%10.4f" % ( data.header.stamp.to_sec(), rospy.Time.now().to_sec() )

        dt = self.get_dt( data.header.stamp )
        if dt < 0:
            return

        imu_data = np.matrix( [ data.vector.x,
                                data.vector.y,
                                data.vector.z ] ).T
        res = self.filter.update_imu( imu_data, self.imu_cov, dt )
        self.publish_state( res, rospy.Time.now() )

    def baro_cb( self, data ):
#        print "baro stamp: %10.4f/%10.4f" % ( data.header.stamp.to_sec(), rospy.Time.now().to_sec() )

        baro_alt = self.baro_filter.update( data.altitude )

        if self.last_baro == None:
            self.last_baro = baro_alt
            self.last_baro_time = data.header.stamp
        else:
            dt = self.get_dt( data.header.stamp )
            if dt < 0:
                return

            baro_dt = ( data.header.stamp - self.last_baro_time ).to_sec()
            self.last_baro_time = data.header.stamp

            dz = baro_alt - self.last_baro
            self.last_baro = baro_alt

            baro_zdot = np.matrix( dz/baro_dt )

            res = self.filter.update_zdot( baro_zdot, self.baro_zdot_cov, dt )
            self.publish_state( res, rospy.Time.now() )

    def publish_state( self, res, timestamp ):
        state = Odometry()
        state.header.stamp = timestamp
        state.header.frame_id = self.world_frame
# We'll fix this bit later
#        state.child_frame_id = self.child_frame
        state.pose.pose.position = Point( *res[0][:3] )
        state.twist.twist.linear = Vector3( *res[0][3:6] )
        zeros_33 = np.zeros( ( 3, 3 ) )

        if self.gps_data != None:
            state.pose.pose.orientation = self.gps_data.pose.pose.orientation
            state.twist.twist.angular = self.gps_data.twist.twist.angular

            orientation_covariance = np.array( self.gps_data.pose.covariance ).reshape( 6, 6 )[3:6,3:6]
            angular_covariance = np.array( self.gps_data.twist.covariance ).reshape( 6, 6 )[3:6,3:6]
        else:
            state.pose.pose.orientation = Quaternion(0,0,0,1)
            orientation_covariance = zeros_33
            angular_covariance = zeros_33

        pose_covariance = res[1][:3,:3]
        twist_covariance = res[1][3:6,3:6]

        full_pose_covariance = np.array( np.hstack( ( np.vstack( ( pose_covariance, zeros_33 ) ),
                                                      np.vstack( ( zeros_33, orientation_covariance ) ) ) ) )
        full_twist_covariance = np.array( np.hstack( ( np.vstack( ( twist_covariance, zeros_33 ) ),
                                                       np.vstack( ( zeros_33, angular_covariance ) ) ) ) )

        state.pose.covariance = list( full_pose_covariance.reshape( 36 ) ) 
        state.twist.covariance = list( full_twist_covariance.reshape( 36 ) ) 

        pose = PoseWithCovarianceStamped( state.header, state.pose )
        twist = TwistWithCovarianceStamped( state.header, state.twist )

        self.state_pub.publish( state )
        self.pose_pub.publish( pose )
        self.twist_pub.publish( twist )

        accel = Vector3Stamped( state.header, Vector3( *res[0][6:9] ) )
        self.accel_pub.publish( accel )

        # publish transforms from /map to /base_footprint
        # and from /map to /base_position

        self.broadcaster.sendTransform((state.pose.pose.position.x,
                                        state.pose.pose.position.y,
                                        0),
                                       (state.pose.pose.orientation.x,
                                        state.pose.pose.orientation.y,
                                        state.pose.pose.orientation.z,
                                        state.pose.pose.orientation.w),
                                       state.header.stamp,
                                       '/robot/base_footprint',
                                       '/map', )

        self.broadcaster.sendTransform((state.pose.pose.position.x, state.pose.pose.position.y, state.pose.pose.position.z ),
                                       (0,0,0,1),
                                       state.header.stamp,
                                       '/robot/base_position',
                                       '/map')



    def sonar_cb( self, data ):
        if data.range > data.max_range:
            self.last_sonar = None
        else:
            if data.range < data.min_range:
                data.range = data.min_range

            try:
                self.listener.waitForTransform( '/robot/base_stabilized', '/robot/sonar_link', data.header.stamp, rospy.Duration(1.0) )
                transform = self.listener.lookupTransform( '/robot/base_stabilized', '/robot/sonar_link', data.header.stamp )
                transform_matrix = tf.transformations.quaternion_matrix( transform[1] )
                point = transform_matrix[:3,:3].dot( np.array( [ data.range, 0, 0 ] ) )
            except (tf.LookupException, tf.Exception,
                    tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn( "kalman_filter: transform exception: %s", str( e ) )
                self.last_sonar = None
            else:
                sonar_range = self.sonar_filter.update( -point[2] )

                # we don't have a last recording (maybe it was beyond the range, or there was a transform error)
                # so don't compute a zdot or run an update
                if self.last_sonar == None:
                    self.last_sonar = sonar_range
                    self.last_sonar_time = data.header.stamp
                else:
                    dt = self.get_dt( data.header.stamp )
                    if dt < 0:
                        return
                    sonar_dt = ( data.header.stamp - self.last_sonar_time ).to_sec()
                    self.last_sonar_time = data.header.stamp

                    dz = sonar_range - self.last_sonar
                    self.last_sonar = sonar_range

                    sonar_zdot = np.matrix( dz/sonar_dt )
#                    print "sonar_zdot %6.1f" % sonar_zdot
                    res = self.filter.update_zdot( sonar_zdot, self.sonar_zdot_cov, dt )
                    self.publish_state( res, rospy.Time.now() )
    def run(self):
        rospy.spin()
    

def main():
    rospy.init_node('kalman_filter' )
    filter = Filter()

    try:
        filter.run()
    except  KeyboardInterrupt as e:
        print "Shutting down"

if __name__  == '__main__':
    main()


        

