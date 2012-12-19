#!/usr/bin/env python

import roslib; roslib.load_manifest("falkor_drivers")
import rospy
import numpy as np
from tf.transformations import *
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from pylab import *
import progressbar

class MagnetometerCalibrator:
    def imu_cb(self, data):
        self.imu_msg = data

    def tilt_compensation(self):
        if self.imu_msg == None:
            rospy.logwarn( "magnetometer_calibrator: imu msg not yet received" )
            return None

        # Rotate magnetic vector per the IMU orientation
        (roll,pitch,yaw) = euler_from_quaternion( ( self.imu_msg.orientation.x,
                                                    self.imu_msg.orientation.y,
                                                    self.imu_msg.orientation.z,
                                                    self.imu_msg.orientation.w ) )
        rotation_matrix = euler_matrix( -roll, -pitch, 0 )[:3,:3]
        vector = np.array( ( self.mag_msg.vector.x,
                             self.mag_msg.vector.y,
                             self.mag_msg.vector.z ) )
        return rotation_matrix.dot( vector )

    def mag_cb(self,data):
        self.mag_msg = data
        
        vector = self.tilt_compensation()
        if vector == None:
            return

        if self.points == None:
            self.points = np.array((vector[:2]))
        else:
            self.points = np.vstack((self.points,vector[:2]))

        self.point_count += 1

    def __init__(self):
        self.mag_msg = None
        self.imu_msg = None
        self.points = None
        self.point_count = 0

        self.mag_sub = rospy.Subscriber( "/magnetic", Vector3Stamped, self.mag_cb )
        self.imu_sub = rospy.Subscriber( "/imu/data", Imu, self.imu_cb )

    def fit_ellipse(self,points):
        # http://nicky.vanforeest.com/misc/fitEllipse/fitEllipse.html
        x = points[:,0][:,np.newaxis]
        y = points[:,1][:,np.newaxis]
        D = np.hstack((x*x, x*y, y*y, x, y, np.ones_like(x)))
        S = np.dot(D.T,D)
        C = np.zeros([6,6])
        C[0,2] = C[2,0] = 2; C[1,1] = -1
        E, V =  eig(np.dot(inv(S), C))
        n = np.argmax(np.abs(E))
        a = V[:,n]
        return a

    def ellipse_center(self,a):
        b,c,d,f,g,a = a[1]/2, a[2], a[3]/2, a[4]/2, a[5], a[0]
        num = b*b-a*c
        x0=(c*d-b*f)/num
        y0=(a*f-b*d)/num
        return np.array([x0,y0])

    def ellipse_angle_of_rotation(self,a):
        b,c,d,f,g,a = a[1]/2, a[2], a[3]/2, a[4]/2, a[5], a[0]
        return 0.5*np.arctan(2*b/(a-c))

    def ellipse_axis_length(self,a):
        b,c,d,f,g,a = a[1]/2, a[2], a[3]/2, a[4]/2, a[5], a[0]
        up = 2*(a*f*f+c*d*d+g*b*b-2*b*d*f-a*c*g)
        down1=(b*b-a*c)*( (c-a)*np.sqrt(1+4*b*b/((a-c)*(a-c)))-(c+a))
        down2=(b*b-a*c)*( (a-c)*np.sqrt(1+4*b*b/((a-c)*(a-c)))-(c+a))
        res1=np.sqrt(up/down1)
        res2=np.sqrt(up/down2)
        return np.array([res1, res2])

    def run(self):
        rospy.loginfo( "Starting magnetometer calibration. Rotate craft now." )

        # Wait until we start to get at least one point
        while self.point_count < 1:
            rospy.sleep(1)

        # now wait two minutes
        progress=progressbar.ProgressBar()
        for i in progress(range(20)):
            rospy.sleep(6)

        rospy.loginfo( "Done collecting data. Calibrating" )
        self.mag_sub.unregister()
        self.imu_sub.unregister()

        ellipse = self.fit_ellipse(self.points)
        center = self.ellipse_center(ellipse)
        angle = self.ellipse_angle_of_rotation(ellipse)
        axes = self.ellipse_axis_length(ellipse)

        rospy.loginfo( "center = %s" % str( center ) )
        rospy.loginfo( "angle = %s" % str( angle ) )
        rospy.loginfo( "axes = %s" % str( ( 1, axes[1]/axes[0] ) ) )

        # plot
        a, b = axes
        R = np.arange(0,2*np.pi,0.01)
        xx = center[0] + a*np.cos(R)*np.cos(angle) - b*np.sin(R)*np.sin(angle)
        yy = center[1] + a*np.cos(R)*np.sin(angle) + b*np.sin(R)*np.cos(angle)

        plot(self.points[:,0],self.points[:,1],'ro')
        plot(xx,yy, color = 'red')
        show()
        
def main():
    rospy.init_node('magnetometer_calibrator')
    calibrator = MagnetometerCalibrator()
    try:
        calibrator.run()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__  == '__main__':
    main()

        
