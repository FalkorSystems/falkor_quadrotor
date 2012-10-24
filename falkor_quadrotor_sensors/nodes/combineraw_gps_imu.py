#!/usr/bin/env python
import roslib
roslib.load_manifest( 'falkor_quadrotor_sensors' )

import rospy
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from nav_msgs.msg import *
import tf

class FalkorCombineGpsImu:
    def __init__( self ):
        self.gps_topic = rospy.get_param( "~gps_fix_topic", "fix_m/pose" )
        self.imu_topic = rospy.get_param( "~imu_topic", "imu" )
        self.world_frame = rospy.get_param( "~world_frame", "/nav" )
        self.gravity = rospy.get_param( "~gravity", 9.82 )

        self.pose_pub = rospy.Publisher( "raw/pose", PoseStamped )
        self.accel_pub = rospy.Publisher( "raw/accel", Vector3Stamped )

        self.listener = tf.TransformListener()

        self.gps_sub = rospy.Subscriber( self.gps_topic, PoseWithCovarianceStamped, self.gps_cb )
        self.imu_sub = rospy.Subscriber( self.imu_topic, Imu, self.imu_cb )

        self.seq = 0

        self.pose = PoseStamped()
        self.pose.pose.orientation = Quaternion( 0, 0, 0, 1 )
        self.accel = Vector3Stamped()

        self.rate = rospy.Rate( rospy.get_param( "~update_rate", 10 ) )

    def gps_cb( self, data ):
        # The gps pose is in the world frame, but double check
        if not data.header.frame_id == self.world_frame:
            raise Exception( "gps data frame does not match world frame" )

        self.pose.pose.position = data.pose.pose.position

    def imu_cb( self, data ):
        # wait for transform
        try:
            self.listener.waitForTransform( data.header.frame_id,
                                            self.world_frame,
                                            data.header.stamp,
                                            rospy.Duration( 4.0 ) )
            quaternion_imu = QuaternionStamped( data.header, data.orientation )
            accel_imu = Vector3Stamped( data.header, data.linear_acceleration )

            # convert from the imu frame to the world frame
            self.pose.pose.orientation = self.listener.transformQuaternion( self.world_frame, quaternion_imu ).quaternion

            self.accel.vector = self.listener.transformVector3( self.world_frame, accel_imu ).vector

            # remove gravity
            self.accel.vector.z -= self.gravity

        except (tf.LookupException, tf.Exception,
                tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn( "combineraw_gps_imu: transform exception: %s", str( e ) )

    def publish( self ):
        self.seq += 1
        self.accel.header = self.pose.header = Header( self.seq, rospy.Time.now(), self.world_frame )
        self.pose_pub.publish( self.pose )
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

            
                                    
            

        
        
