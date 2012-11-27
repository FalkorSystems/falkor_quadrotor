#!/usr/bin/env python

import roslib; roslib.load_manifest("falkor_drivers")
import rospy

import pwm_pololu_driver
import pwm_input_driver
from serial import *

def main():
    rospy.init_node('pwm_io_driver')

    port_name = rospy.get_param( "~port", "/dev/ttyO1" )
    baud = rospy.get_param( "~baud", 115200 )
    timeout = rospy.get_param( "~timeout", 5.0 )

    port = Serial(port_name, baud, timeout=timeout * 0.5)
    input_driver = pwm_input_driver.PwmInputDriver( port )
    output_driver = pwm_pololu_driver.PwmOutputDriver( port )

    input_driver.run()

if __name__  == '__main__':
    main()
