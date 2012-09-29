#!/usr/bin/env python
import roslib
roslib.load_manifest('falkor_quadrotor_navigate')

import rospy
import tf
import numpy
from geometry_msgs.msg import *
from std_msgs.msg import *

class FalkorQuadrotorTargetPt:
    def __init__( self ):
        self.seq = 0
        self.targetpt_pub = rospy.Publisher( "beacon/target_point", PointStamped )
        self.cmd_vel_sub = rospy.Subscriber( "cmd_vel", Twist, self.update_targetpt )

        self.horizontal_distance = 3.0
        self.bearing = 0.0
        self.vertical_distance = 3.0
        self.last_update = rospy.Time.now()
        self.rate = rospy.Rate( 10.0 )

    def update_targetpt( self, data ):
        time_now = rospy.Time.now()
        dt = (time_now - self.last_update).to_sec()
        self.last_update = time_now

        bearing_chg = data.angular.z * dt
        horizontal_chg = data.linear.x * dt
        vertical_chg = data.linear.z * dt

        self.bearing += bearing_chg
        self.horizontal_distance += horizontal_chg
        self.vertical_distance += vertical_chg

    def publish_target_point( self ):
        self.seq += 1
        z = self.vertical_distance
        y = numpy.sin( self.bearing ) * self.horizontal_distance
        x = numpy.cos( self.bearing ) * self.horizontal_distance

        pub_data = PointStamped( Header( self.seq, rospy.Time.now(), '/beacon/estimate/base_position' ),
                                 Point( x, y, z ) )
        self.targetpt_pub.publish( pub_data )

    def run( self ):
        while not rospy.is_shutdown():
            self.seq += 1

            self.publish_target_point()
            self.rate.sleep()

def main():
    rospy.init_node('falkor_quadrotor_targetpt')
    target_pt = FalkorQuadrotorTargetPt()
    try:
        target_pt.run()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__  == '__main__':
    main()

        
    
