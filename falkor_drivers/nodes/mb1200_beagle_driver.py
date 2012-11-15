#!/usr/bin/env python
# This driver uses the analog pin, assuming that the voltage
# divider is setup properly so that the max input is 1.8V into 
# the beaglebone

import roslib; roslib.load_manifest("falkor_drivers")
import rospy
from sensor_msgs.msg import *
from std_msgs.msg import *
import numpy as np

class Mb1200Driver:
    def __init__(self, pin=0):
        self.file = '/sys/devices/platform/omap/tsc/ain' + ( pin + 1 )

        # 99ms update rate
        self.update_timer = ropsy.Timer( rospy.Duration( 0.099 ), self.update_cb )
        self.sonar_topic = rospy.param( "~sonar_topic", "sonar" )
        self.tf_prefix = rospy.param( "~tf_prefix", "" )
        self.sonar_frame = self.tf_prefix + "/" + rospy.get_param( "~sonar_frame", "sonar_link" )
        self.range_pub = rospy.Publisher( self.sonar_topic, Range )

    def update_cb( self, data ):
        # read from file
        f = open( self.file, 'r' )
        data = f.read()
        number = int( data )
        
        # 3.3V yields 3.2mV/cm so 1.8V should yield 1.745 mV/cm
        mV = number / 4096.0 * 1800.0
        m = mV/174.5
        msg = Range()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.sonar_frame
        msg.radiation_type = Range.ULTRASOUND
        msg.min_range = 0.2
        msg.max_range = 6
        msg.range = m
        self.range_pub( msg )

    def run(self):
        rospy.spin()

def main():
    rospy.init_node('mb1200_beagle_driver')
    driver = Mb1200Driver()
    try:
        driver.run()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__  == '__main__':
    main()
