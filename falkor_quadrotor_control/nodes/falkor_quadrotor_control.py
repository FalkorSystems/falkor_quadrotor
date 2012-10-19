#!/usr/bin/env python
import roslib
roslib.load_manifest('falkor_quadrotor_control')

import rospy
import pid
import tf
from geometry_msgs.msg import *

class FalkorQuadrotorControl:
    def __init__( self ):
        self.yaw_pid = pid.PidController( -0.5, 0, 0 )
        self.x_pid = pid.PidController( -2.5, 0, 0 )
        self.y_pid = pid.PidController( -2.5, 0, 0 )
        self.z_pid = pid.PidController( -1.25, 0, 0 )

        self.target_pose_sub = rospy.Subscriber( '/navigate/robot/target_pose', PoseStamped,
                                                 self.target_pose_update )
        self.cmd_vel_pub = rospy.Publisher( '/robot/cmd_vel', Twist )
        self.last_update = rospy.Time.now()
        self.cmd_gimbal_pub = rospy.Publisher( '/robot/cmd_gimbal', Point )
        self.listener = tf.TransformListener()

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
            self.listener.waitForTransform( '/ekf/robot/base_stabilized',
                                            '/ekf/robot/base_link',
                                            data.header.stamp,
                                            rospy.Duration( 4.0 ) )

            (trans, rot) = self.listener.lookupTransform( '/ekf/robot/base_stabilized',
                                                          '/ekf/robot/base_link',
                                                          data.header.stamp )

            stabilize_euler = tf.transformations.euler_from_quaternion( rot )


            # The gimbal pitch is the pitch of the euler above plus the pitch between link
            # and stabilized

            gimbal_pitch = euler[1] - stabilize_euler[1]
            gimbal_roll = stabilize_euler[0]

            gimbal_cmd = Point( gimbal_pitch, gimbal_roll, 0 )

            self.cmd_gimbal_pub.publish( gimbal_cmd )
        except (tf.LookupException, tf.Exception,
                tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn( "control: transform exception: %s", str( e ) )
            return
 

    def run( self ):
        rospy.spin()
        
def main():
    rospy.init_node('falkor_quadrotor_control')
    control = FalkorQuadrotorControl()

    # wait a minute before starting
    rospy.sleep( rospy.Duration( 1.0 ) )
    try:
        control.run()
    except KeyboardInterrupt:
        print "Shutting down"
if __name__  == '__main__':
    main()

