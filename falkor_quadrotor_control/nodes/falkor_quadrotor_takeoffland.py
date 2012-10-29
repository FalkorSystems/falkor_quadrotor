#!/usr/bin/env python
import roslib
roslib.load_manifest('falkor_quadrotor_control')

import rospy
import pid
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from std_srvs.srv import *
import tf

class FalkorQuadrotorTakeoffLand:
    def __init__( self ):
        self.ground_height = rospy.get_param( "~ground_height", 0.15 )
        self.takeoff_height = rospy.get_param( "~takeoff_height", 2.0 )
        self.takeoff_tol = rospy.get_param( "~takeoff_tol", 0.25 )
        self.land_tol = rospy.get_param( "~land_tol", 0.05 )
        self.listener = tf.TransformListener()

        self.height = None

        rospy.wait_for_service( "on" )
        self.on_service = rospy.ServiceProxy( "on", Empty )

        rospy.wait_for_service( "off" )
        self.off_service = rospy.ServiceProxy( "off", Empty, self.takeoff )

        self.takeoff_service = rospy.Service( "takeoff", Empty, self.takeoff )
        self.land_service = rospy.Service( "land", Empty, self.land )

        self.cmd_vel_pub = rospy.Publisher( 'cmd_vel', Twist )

        self.sonar_sub = rospy.Subscriber( "sonar_height", Range, self.sonar_cb )


    def sonar_cb( self, data ):
        point_msg = PointStamped( data.header, Point( data.range, 0, 0 ) )
        try:
            self.listener.waitForTransform( '/robot/base_stabilized',
                                            point_msg.header.frame_id,
                                            rospy.Time.now(),
                                            rospy.Duration( 4.0 ) )
            point_transformed = self.listener.transformPoint( '/robot/base_stabilized', point_msg )
        except (tf.LookupException, tf.Exception,
                tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn( "takeoffland: transform exception: %s", str( e ) )
            return
        self.height = -point_transformed.point.z

    def control( self, target_height, height_tol, pid_params ):
        my_pid = pid.PidController( *pid_params )
        my_pid.setPoint = target_height
        rate = rospy.Rate( 10 )
        last_time = rospy.Time.now()
        been_good = 0
        while been_good < 100:
            rate.sleep()
            time = rospy.Time.now()
            dt = ( last_time - time ).to_sec()
            cmd_vel = Twist()
            cmd_vel.linear.z = my_pid.get_output( self.height, dt )
            self.cmd_vel_pub.publish( cmd_vel )

            if abs( self.height - target_height ) < height_tol:
                been_good += 1
            else:
                been_good = 0

    def takeoff( self, req ):
        self.on_service()
        self.control( self.takeoff_height, self.takeoff_tol, ( 2.0, 0, 1.0 ) )
        return EmptyResponse()

    def land( self, req ):
        self.control( self.ground_height, self.land_tol, ( 0.5, 0, 0.5 ) )
        self.off_service()
        return EmptyResponse()

    def run( self ):
        rospy.spin()
        
def main():
    rospy.init_node('falkor_quadrotor_takeoffland')
    control = FalkorQuadrotorTakeoffLand()

    try:
        control.run()
    except ( KeyboardInterrupt, rospy.Exceptions.ROSInterruptException ) as e:
        print "Shutting down"

if __name__  == '__main__':
    main()
