#!/usr/bin/env python

import roslib; roslib.load_manifest("falkor_drivers")
import rospy
from serial import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from falkor_msgs.msg import *
import numpy as np
from i2c_sensors import *

class I2CDriver:
    def __init__(self):
        self.i2c_number = 3
        self.i2c = I2CSensors( self.i2c_number )

        self.mag_topic = rospy.get_param( "~mag_topic", "magnetic" )
        self.imu_topic = rospy.get_param( "~imu_topic", "raw_imu" )
        self.baro_topic = rospy.get_param( "~baro_topic", "altimeter" )

        self.tf_prefix = rospy.get_param( "~tf_prefix", "" )
        self.mag_frame = self.tf_prefix + "/" + rospy.get_param( "~mag_frame", "magnetometer" )
        self.imu_frame = self.tf_prefix + "/" + rospy.get_param( "~imu_frame", "imu" )
        self.baro_frame = self.tf_prefix + "/" + rospy.get_param( "~baro_frame", "altimeter" )

        self.mag_pub = rospy.Publisher( self.mag_topic, Vector3Stamped )
        self.imu_pub = rospy.Publisher( self.imu_topic, Imu )
        self.baro_pub = rospy.Publisher( self.baro_topic, Altimeter )

        self.baro_timer = rospy.Timer( rospy.Duration( 1/float(rospy.get_param( "~baro_rate", 100) ) ),
                                       self.baro_cb )
        self.imu_timer = rospy.Timer( rospy.Duration( 1/float(rospy.get_param( "~imu_rate", 100 ) )),
                                      self.imu_cb )
        self.mag_timer = rospy.Timer( rospy.Duration( 1/float(rospy.get_param( "~mag_rate", 15 )) ),
                                      self.mag_cb )

    def baro_cb( self, event ):
        baro_data = self.i2c.baro_read()
        msg = Altimeter()
        msg.header.frame_id = self.baro_frame
	msg.header.stamp = rospy.Time.now()
        msg.altitude = baro_data[2]
        msg.pressure = baro_data[1]
        self.baro_pub.publish( msg )

    def mag_cb( self, event ):
        mag_data = self.i2c.mag_read()
        msg = Vector3Stamped()
        msg.header.frame_id = self.mag_frame
	msg.header.stamp = rospy.Time.now()
        msg.vector = Vector3( *mag_data )

        self.mag_pub.publish( msg )

    def imu_cb( self, event ):
        accel_data = self.i2c.accel_read()
        gyro_data = self.i2c.gyro_data()
        msg = Imu()

        msg.header.frame_id = self.imu_frame
	msg.header.stamp = rospy.Time.now()
        msg.orientation = Quaternion( 0, 0, 0, 1.0 )
        msg.orientation_covariance = [0]*9

        gyro_data = np.array( gyro_data ) / 14.375 / 180 * np.pi
        msg.angular_velocity = Vector3( *gyro_data )
	msg.angular_velocity.y = -msg.angular_velocity.y
	msg.angular_velocity.z = -msg.angular_velocity.z
        
        # We need to figure this out
        msg.angular_velocity_covariance = [0] * 9

        accel_data = np.array( accel_data ) / 256.0 * 9.82
        msg.linear_acceleration = Vector3( *accel_data )

        # Figure this out too
        msg.linear_acceleration_covariance = [0]*9

        self.imu_pub.publish(msg)

    def run(self):
	rospy.spin()

def main():
    rospy.init_node('i2c_driver')
    driver = I2CDriver()
    try:
        driver.run()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__  == '__main__':
    main()

