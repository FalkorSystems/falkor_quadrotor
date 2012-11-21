#!/usr/bin/env python

import roslib; roslib.load_manifest( 'falkor_quadrotor_teleop' )
import rospy

from falkor_msgs.msg import *
from sensor_msgs.msg import Joy

class PwmTeleopJoy:
    def __init__( self ):
        self.throttle_channel = rospy.get_param( "~throttle_channel", 1 )
        self.yaw_channel = rospy.get_param( "~yaw_channel", 2 )
        self.pitch_channel = rospy.get_param( "~pitch_channel", 3 )
        self.roll_channel = rospy.get_param( "~roll_channel", 4 )

        self.pwm_min = rospy.get_param( "~pwm_min", 1000 )
        self.pwm_max = rospy.get_param( "~pwm_max", 2000 )
        self.pwm_range = self.pwm_max - self.pwm_min

        self.pwm_pub = rospy.Publisher( "pwm", Pwm )

        self.joy_sub = rospy.Subscriber( "joy", Joy, self.callback_joy )

    def to_pwm( self, value, min_value = -1, max_value = 1 ):
        # convert a float from -1 to 1 into an int from pwm_min to pwm_max

        # limit to [-100,100]
        value = min( max( value, min_value ), max_value )
        value_to_pwm = self.pwm_range / ( max_value - min_value )
        
        pwm_value = ( value - min_value ) * value_to_pwm + self.pwm_min
        return int( pwm_value )

    def callback_joy( self, data ):
        pwm_msg = Pwm()
        pwm_msg.pwm = [0] * 6
        pwm_msg.pwm[self.yaw_channel-1] = self.to_pwm( data.axes[2] )
        pwm_msg.pwm[self.throttle_channel-1] = self.to_pwm( data.axes[3] )
        pwm_msg.pwm[self.pitch_channel-1] = self.to_pwm( data.axes[1] )
        pwm_msg.pwm[self.roll_channel-1] = self.to_pwm( data.axes[0] )

        self.pwm_pub.publish( pwm_msg )

def main():
  rospy.init_node( 'pwm_teleop_joy' )

  PwmTeleopJoy()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

if __name__ == '__main__':
    main()
            
    

