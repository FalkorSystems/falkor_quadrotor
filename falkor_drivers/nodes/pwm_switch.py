#!/usr/bin/env python

import roslib; roslib.load_manifest("falkor_drivers")
import rospy
from falkor_msgs.msg import Pwm

# If channel 5 of pwm_in > 1500 then use pwm_in
# otherwise use pwm_cmd

class PwmSwitch:
    def __init__(self):
        self.pwm_in_topic = rospy.get_param( "~pwm_in_topic", "pwm_in" )
        self.pwm_cmd_topic = rospy.get_param( "~pwm_cmd_topic", "pwm_cmd" )
        self.pwm_out_topic = rospy.get_param( "~pwm_out_topic", "pwm_out" )

        self.pwm_cmd = Pwm([0] * 5)
        self.pwm_in = Pwm([0] * 5)

	# Chan 5 fix is what we use to set the control mode switch
	# for the DJI Naza 1870 means GPS
	self.chan_5_fix = rospy.get_param( "~chan_five_fix", 1870 )
        self.rate = rospy.Rate(50)

        self.pwm_out_pub = rospy.Publisher( self.pwm_out_topic, Pwm )
        self.pwm_in_sub = rospy.Subscriber( self.pwm_in_topic, Pwm, self.pwm_in_cb )
        self.pwm_cmd_sub = rospy.Subscriber( self.pwm_cmd_topic, Pwm, self.pwm_cmd_cb )

    def pwm_in_cb(self, data):
        self.pwm_in.pwm = list( data.pwm )

    def pwm_cmd_cb(self, data):
        self.pwm_cmd.pwm = list( data.pwm )

    def run(self):
        while not rospy.is_shutdown():
            if self.pwm_in == None:
                pass
            elif self.pwm_in.pwm[4] > 1500:            
		pwm_out = self.pwm_in
		pwm_out.pwm[4] = self.chan_5_fix
                self.pwm_out_pub.publish( pwm_out )
            else:
		pwm_out = self.pwm_cmd
		pwm_out.pwm[4] = self.chan_5_fix
                self.pwm_out_pub.publish( pwm_out )
            self.rate.sleep()

def main():
    rospy.init_node('pwm_switch')
    driver = PwmSwitch()
    try:
        driver.run()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__  == '__main__':
    main()


