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

        self.mag_center = rospy.get_param( "~mag_calib/center", { 'x': 0, 'y': 0 } )
        self.mag_axes = rospy.get_param( "~mag_calib/axes", { 'x': 1, 'y': 1 } )
        self.mag_angle = rospy.get_param( "~mag_calib/angle", 0 )

	print self.mag_center, self.mag_axes, self.mag_angle 

        self.mag_cos_angle = np.cos( - self.mag_angle )
        self.mag_sin_angle = np.sin( - self.mag_angle )

        self.baro_timer = rospy.Timer( rospy.Duration( 1/float(rospy.get_param( "~baro_rate", 25) ) ),
                                       self.baro_cb )
        self.imu_timer = rospy.Timer( rospy.Duration( 1/float(rospy.get_param( "~imu_rate", 100 ) )),
                                      self.imu_cb )
        self.mag_timer = rospy.Timer( rospy.Duration( 1/float(rospy.get_param( "~mag_rate", 15 )) ),
                                      self.mag_cb )

        self.pwm_sub = rospy.Subscribe( "pwm_out", Pwm, self.pwm_cb )

    def pwm_cb( self, data ):
        for i in range(0,len(data.pwm)):
            rospy.logdebug( "sending %d to %d" % (data.pwm[i],i) )
            self.i2c.write_pwm(i,data.pwm[i])

    def baro_cb( self, event ):
        # baro_read w/ oss=3 should give us 0.25m RMS accuracy
        baro_data = self.i2c.baro_read(3)
        if baro_data == None:
            return

        msg = Altimeter()
        msg.header.frame_id = self.baro_frame
	msg.header.stamp = rospy.Time.now()
        msg.altitude = baro_data[2]
        msg.pressure = baro_data[1]
        self.baro_pub.publish( msg )

    def mag_cb( self, event ):
        mag_data = self.i2c.mag_read()
        if mag_data == None:
            return

        # transform for calibration

        # translate
        mag_data[0] -= self.mag_center['x']
        mag_data[1] -= self.mag_center['y']

        # rotate
        mag_data[0] = mag_data[0] * self.mag_cos_angle + mag_data[1] * self.mag_sin_angle
        mag_data[1] = - mag_data[1] * self.mag_sin_angle + mag_data[1] * self.mag_cos_angle

        # scale
        mag_data[0] /= self.mag_axes['x']
        mag_data[1] /= self.mag_axes['y']

        msg = Vector3Stamped()
        msg.header.frame_id = self.mag_frame
	msg.header.stamp = rospy.Time.now()
        msg.vector = Vector3( *mag_data )

        self.mag_pub.publish( msg )

    def imu_cb( self, event ):
        accel_data = self.i2c.accel_read()
        gyro_data = self.i2c.gyro_read()
        if accel_data == None and gyro_data == None:
            return

        msg = Imu()

        msg.header.frame_id = self.imu_frame
	msg.header.stamp = rospy.Time.now()
        msg.orientation = Quaternion( 0, 0, 0, 1.0 )
        msg.orientation_covariance = [0]*9

        if gyro_data != None:
            msg.angular_velocity = Vector3( *gyro_data )
            msg.angular_velocity.y = -msg.angular_velocity.y
            msg.angular_velocity.z = -msg.angular_velocity.z
        
        # We need to figure this out
        msg.angular_velocity_covariance = [0] * 9

        if accel_data != None:
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

