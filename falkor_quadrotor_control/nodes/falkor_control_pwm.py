#!/usr/bin/env python
import roslib
roslib.load_manifest('falkor_quadrotor_control')

import rospy
import pid
import tf
from geometry_msgs.msg import *
from std_srvs.srv import *
from falkor_msgs.msg import *
from nav_msgs.msg import *

class FalkorControlPwm:
    def __init__( self ):
        self.on = False

        self.angular_z_pid = pid.PidController( 1, 0, 0 )
        self.linear_x_pid = pid.PidController( 1, 0, 0 )
        self.linear_y_pid = pid.PidController( 1, 0, 0 )
        self.linear_z_pid = pid.PidController( 1, 0, 0 )

        self.pwm_pub = rospy.Publisher( "pwm_cmd", Pwm )

        self.on_service = rospy.Service( 'on', Empty, self.turn_on )
        self.off_service = rospy.Service( 'off', Empty, self.turn_off )

        self.throttle_channel = rospy.get_param( "~throttle_channel", 3 )
        self.yaw_channel = rospy.get_param( "~yaw_channel", 4 )
        self.pitch_channel = rospy.get_param( "~pitch_channel", 2 )
        self.roll_channel = rospy.get_param( "~roll_channel", 1 )

        self.gimbal_pitch_channel = rospy.get_param( "~gimbal_pitch_channel", 6 )
        self.gimbal_roll_channel = rospy.get_param( "~gimbal_roll_channel", 7 )

        self.num_channels = max( self.throttle_channel,
                                 self.yaw_channel,
                                 self.pitch_channel,
                                 self.roll_channel,
                                 self.gimbal_roll_channel,
                                 self.gimbal_pitch_channel )

        self.cmd_vel = Twist()
        self.cmd_gimbal = Gimbal()
        self.state = Odometry()
        self.pwm_msg = Pwm( [1500]*self.num_channels )

        self.frame_id = rospy.get_param( "~frameId", "base_link" )
        self.tf_prefix = rospy.get_param( "~tf_prefix", "" )
        self.full_frame_id = self.tf_prefix + "/" + self.frame_id

        self.pwm_min = rospy.get_param( "~pwm_min", 1000 )
        self.pwm_max = rospy.get_param( "~pwm_max", 2000 )
        self.pwm_range = self.pwm_max - self.pwm_min

        self.update_rate = rospy.get_param( "~update_rate", 10 )
        self.rate = rospy.Rate( self.update_rate )

        self.cmd_vel_sub = rospy.Subscriber( "cmd_vel", Twist, self.cmd_vel_cb )
        self.cmd_gimbal_sub = rospy.Subscriber( "cmd_gimbal", Gimbal, self.cmd_gimbal_cb )
        self.state_sub = rospy.Subscriber( "state", Odometry, self.state_cb )


    def turn_on( self, req ):
        if self.on:
            return EmptyResponse()

        pwm_cmd = Pwm( [1500] * self.num_channels )

        # DJI on command
        # Bring the left stick to lower left, the right stick to lower right
        # Left stick is channels 3 (throttle) and 4 (yaw)
        # Right stick is channels 2 (pitch) and 1 (roll)
        # channel 5 we have to send but can be 0
        pwm_cmd.pwm[self.throttle_channel-1] = 1000
        pwm_cmd.pwm[self.yaw_channel-1] = 1000
        pwm_cmd.pwm[self.roll_channel-1] = 2000
        pwm_cmd.pwm[self.pitch_channel-1] = 1000

        self.pwm_pub.publish( pwm_cmd )

        # Wait a second
        rospy.sleep( 1 )

        # Return everything to the center
        pwm_cmd.pwm[self.throttle_channel-1] = 1500
        pwm_cmd.pwm[self.yaw_channel-1] = 1500
        pwm_cmd.pwm[self.roll_channel-1] = 1500
        pwm_cmd.pwm[self.pitch_channel-1] = 1500

        self.pwm_pub.publish( pwm_cmd )

        self.on = True
        return EmptyResponse()

    def turn_off( self, req ):
        if not self.on:
            return EmptyResponse()

        pwm_cmd = Pwm( [1500] * self.num_channels )
        # Send throttle to 0, everything else to the center
        pwm_cmd.pwm[self.throttle_channel-1] = 1000
        pwm_cmd.pwm[self.yaw_channel-1] = 1500
        pwm_cmd.pwm[self.roll_channel-1] = 1500
        pwm_cmd.pwm[self.pitch_channel-1] = 1500

        self.pwm_pub.publish( pwm_cmd )

        self.on = False
        return EmptyResponse()

    def to_pwm( self, value, min_value = -100, max_value = 100 ):
        # convert a float from -100 to 100 into an int from pwm_min to pwm_max

        # limit to [-100,100]
        value = min( max( value, min_value ), max_value )
        value_to_pwm = self.pwm_range / ( max_value - min_value )
        
        pwm_value = ( value - min_value ) * value_to_pwm + self.pwm_min
        return int( pwm_value )
        
    def cmd_vel_cb( self, data ):
        self.cmd_vel = data

    def cmd_gimbal_cb( self, data ):
        self.cmd_gimbal = data

    def state_cb( self, data ):
        self.state = data

    def wait_for_on( self ):
        self.last_time = rospy.Time.now()

    def control( self ):
        now = rospy.Time.now()
        dt = ( self.last_time - now ).to_sec()
        self.last_time = now

        self.angular_z_pid.setSetPoint( self.cmd_vel.angular.z )
        self.linear_x_pid.setSetPoint( self.cmd_vel.linear.x )
        self.linear_y_pid.setSetPoint( self.cmd_vel.linear.y )
        self.linear_z_pid.setSetPoint( self.cmd_vel.linear.z )

        # transform data into my own frame
        try:
            self.listener.waitForTransform( self.full_frame_id,
                                            self.state.header.frame_id,
                                            rospy.Time(0),
                                            rospy.Duration( 4.0 ) )

            my_frame_state = tf.transformOdometry( self.full_frame_id,
                                                   self.state )

        except (tf.LookupException, tf.Exception,
                tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn( "falkor_control_pwm: transform exception: %s", str( e ) )
            return

        self.pwm_msg.pwm[self.yaw_channel-1] = self.to_pwm( 
            self.angular_z_pid.get_output( 
                my_frame_state.twist.twist.angular.z,
                dt ) )
        self.pwm_msg.pwm[self.throttle_channel-1] = self.to_pwm(
            self.linear_z_pid.get_output(
                my_frame_state.twist.twist.linear.z,
                dt ) )
        self.pwm_msg.pwm[self.pitch_channel-1] = self.to_pwm(
            self.linear_x_pid.get_output(
                my_frame_state.twist.twist.linear.x,
                dt ) )
        self.pwm_msg.pwm[self.roll_channel-1] = self.to_pwm(
            self.linear_y.pid.get_output(
                my_frame_state.twist.twist.linear.y,
                dt ) )

        self.pwm_msg.pwm[self.gimbal_roll_channel-1] = self.to_pwm(
            self.cmd_gimbal.roll, -np.pi/2, np.pi/2 )
        self.pwm_msg.pwn[self.gimbal_pitch_channel-1] = self.to_pwm(
            self.cmd_gimbal.pitch, -np.pi/2, np.pi/2 )

        self.pwm_pub.publish( self.pwm_msg )

    def run( self ):
        while not rospy.is_shutdown():
            if self.on:
                self.control()
            else:
                self.wait_for_on()

            self.rate.sleep()


def main():
    rospy.init_node('falkor_control_pwm')
    control = FalkorControlPwm()

    try:
        control.run()
    except KeyboardInterrupt as e:
        print "Shutting down"

if __name__  == '__main__':
    main()


        


