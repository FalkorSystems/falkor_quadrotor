#!/usr/bin/env python

import roslib; roslib.load_manifest("falkor_drivers")
import rospy
from serial import *
from falkor_msgs.msg import Pwm

class PwmInputDriver:
    def __init__(self, pin=0):
        self.pwm_topic = rospy.get_param( "~input_topic", "pwm" )
        self.pwm_pub = rospy.Publisher( self.pwm_topic, Pwm )
        self.port_name = rospy.get_param( "~port", "/dev/ttyUSB1" )
        self.baud = rospy.get_param( "~baud", 115200 )
        self.timeout = rospy.get_param( "~timeout", 5.0 )

        self.port = Serial(self.port_name, self.baud, timeout=self.timeout * 0.5)

    def parse_line(self, line):
        split = line.split(',')
        split_ints = [int(a) for a in split]
        return split_ints

    def run(self):
        while not rospy.is_shutdown():
            line = self.port.readline()

            try:
                pwm_data = self.parse_line(line)
            except Exception as e:
                rospy.logwarn( "Invalid pwm data line: "  + str(e) )
            else:
                pwm_msg = Pwm( pwm_data )
                self.pwm_pub.publish( pwm_msg )

def main():
    rospy.init_node('pwm_input_driver')
    driver = PwmInputDriver()
    try:
        driver.run()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__  == '__main__':
    main()
