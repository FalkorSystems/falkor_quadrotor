#!/usr/bin/env python
import roslib
roslib.load_manifest('falkor_quadrotor_control')

import rospy
import pid
import tf
from geometry_msgs.msg import *
from std_srvs.srv import *

class FalkorQuadrotorControl:
    def __init__( self ):
        self.yaw_pid = pid.PidController( -0.5, 0, 0 )
        self.x_pid = pid.PidController( -1.25, 0, 0.5 )
        self.y_pid = pid.PidController( -1.25, 0, 0.5 )
        self.z_pid = pid.PidController( -1.25, 0, 0 )

        print "waiting for services"
        rospy.wait_for_service( "/robot/takeoff" )
        self.on_service = rospy.ServiceProxy( "/robot/takeoff", Empty )

        rospy.wait_for_service( "/robot/land" )
        self.off_service = rospy.ServiceProxy( "/robot/land", Empty )

        self.target_pose_sub = rospy.Subscriber( '/navigate/robot/target_pose', PoseStamped,
                                                 self.target_pose_update )
        self.cmd_vel_pub = rospy.Publisher( '/robot/cmd_vel', Twist )
        self.last_update = rospy.Time.now()
        self.cmd_gimbal_pub = rospy.Publisher( '/robot/cmd_gimbal', Gimbal )
        self.listener = tf.TransformListener()

    def on( self ):
        req = Empty()
        self.on_service()
        self.cmd_vel_pub.publish( Twist() )

    def off( self ):
        req = Empty()
        self.off_service()

    def target_pose_update( self, data ):
        dt = ( data.header.stamp - self.last_update ).to_sec()
        self.last_update = data.header.stamp

        euler = tf.transformations.euler_from_quaternion( [ data.pose.orientation.x,
                                                            data.pose.orientation.y,
                                                            data.pose.orientation.z,
                                                            data.pose.orientation.w ] )
        twist_cmd = Twist()

        twist_cmd.angular.z = self.yaw_pid.get_output( euler[2], dt )
        twist_cmd.linear.x = self.x_pid.get_output( data.pose.position.x, dt )
        twist_cmd.linear.y = self.y_pid.get_output( data.pose.position.y, dt )
        twist_cmd.linear.z = self.z_pid.get_output( data.pose.position.z, dt )

        self.cmd_vel_pub.publish( twist_cmd )

        try:
            # Get the transform from base_stabilized to base_link
            # so we know what angle to roll the gimbal at
            self.listener.waitForTransform( '/robot/base_stabilized',
                                            '/robot/base_link',
                                            data.header.stamp,
                                            rospy.Duration( 4.0 ) )

            (trans, rot) = self.listener.lookupTransform( '/robot/base_stabilized',
                                                          '/robot/base_link',
                                                          data.header.stamp )

            stabilize_euler = tf.transformations.euler_from_quaternion( rot )


            # The gimbal pitch is the pitch of the euler above plus the pitch between link
            # and stabilized

            gimbal_pitch = euler[1] - stabilize_euler[1]
            gimbal_roll = stabilize_euler[0]

            gimbal_cmd = Gimbal( gimbal_roll, gimbal_pitch, 0 )

            self.cmd_gimbal_pub.publish( gimbal_cmd )
        except (tf.LookupException, tf.Exception,
                tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn( "control: transform exception: %s", str( e ) )
            return
 
    def run( self ):
        print "turning on"
        self.on()
        rospy.spin()
        
def main():
    rospy.init_node('falkor_quadrotor_control')
    control = FalkorQuadrotorControl()

    # wait a minute before starting
    print "waiting 5 seconds"
    rospy.sleep( rospy.Duration( rospy.get_param( "~calibration_pause", 5 ) ) )

    try:
        control.run()
    except ( KeyboardInterrupt, rospy.Exceptions.ROSInterruptException ) as e:
        # send a 'turn off command'
        print "turning off because of", e
        control.off()
        print "Shutting down"

if __name__  == '__main__':
    main()

