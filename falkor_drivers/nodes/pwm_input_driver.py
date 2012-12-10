#!/usr/bin/env python

import roslib; roslib.load_manifest("falkor_drivers")
import rospy
from serial import *
from falkor_msgs.msg import Pwm

class PwmInputDriver:
    def __init__(self, port):
        self.port = port
        self.pwm_topic = rospy.get_param( "~pwm_in_topic", "pwm_in" )
        self.pwm_pub = rospy.Publisher( self.pwm_topic, Pwm )

    def split_line(self, line):
        split = line.split(',')
        split_ints = [int(a) for a in split]
        return split_ints

    def parse_line( self, line ):
        try:
            pwm_data = self.split_line(line)
        except Exception as e:
            rospy.logwarn( "Invalid pwm data line: "  + str(e) )
        else:
            pwm_msg = Pwm( pwm_data )
            try:
                self.pwm_pub.publish( pwm_msg )
            except rospy.exceptions.ROSSerializationException as e:
                rospy.logwarn( "while publishing %s: %s" % ( pwm_msg, str(e) ) )

    def run(self):
        while not rospy.is_shutdown():
            line = self.port.readline()
            self.parse_line( line )

def main():
    rospy.init_node('pwm_input_driver')

    port_name = rospy.get_param( "~port", "/dev/ttyUSB0" )
    baud = rospy.get_param( "~baud", 115200 )
    timeout = rospy.get_param( "~timeout", 5.0 )

    port = Serial(port_name, baud, timeout=timeout * 0.5)

    driver = PwmInputDriver(port)
    try:
        driver.run()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__  == '__main__':
    main()
