#!/usr/bin/env python
#
# we listen to a PointStamped
# which gives us a desired position that the quad should have relative to
# the beacon, this is going to be in base_position frame of the beacon
#
# We then transform the PointStamped into the robot's base_stabilized
# frame
#
# Then we publish a Pose with the new desired position and orientation of the robot
# relative to itself
#
# The desired orientation should be given the robot's position to the 
# beacon's position, not the target_point to the beacon
# 
# This gives us where the robot needs to move to, relative to itself
#
import roslib
roslib.load_manifest('falkor_quadrotor_navigate')

import rospy
import tf
import numpy
from geometry_msgs.msg import *
from std_msgs.msg import *

class FalkorQuadrotorNav:
    def point_to_quaternion( self, point ):
        bearing = numpy.arctan2( point.y, -point.x )
        inclination = numpy.arctan2( point.z, numpy.sqrt( numpy.square( point.x ) +
                                                          numpy.square( point.y ) ) )

        q = Quaternion( *tuple( tf.transformations.quaternion_from_euler( 0.0, inclination, -bearing ) ) )
        return q

    def update_relative_pose( self, data ):
        self.relative_pose = PoseStamped()
        self.relative_pose.pose.position = data.point
        self.relative_pose.header = data.header

        self.relative_pose.pose.orientation = self.point_to_quaternion( data.point )

    def __init__( self ):
        self.seq = 0
        self.listener = tf.TransformListener()

        self.subscriber = rospy.Subscriber( 'beacon/target_point', PointStamped,
                                            self.update_relative_pose )
        self.nav_target = rospy.Publisher( 'robot/target_pose', PoseStamped )

        relative_point = PointStamped( Header( 0, rospy.Time.now(),
                                              '/ekf/beacon/base_position' ),
                                      Point( 1, 0.5, 0.25 ) )

        self.update_relative_pose( relative_point )
        self.publish_target_pose()

        self.rate = rospy.Rate(10.0)


    def publish_target_pose( self ):
        relpose_cached = self.relative_pose

        try:
#            print "waiting for transform"
            self.listener.waitForTransform( '/ekf/robot/base_stabilized',
                                            '/ekf/beacon/base_position',
                                            relpose_cached.header.stamp,
                                            rospy.Duration( 4.0 ) )

            (trans,rot) = self.listener.lookupTransform( '/ekf/robot/base_stabilized',
                                                         '/ekf/beacon/base_position',
                                                         relpose_cached.header.stamp )
#            print "got transform"
        except (tf.LookupException, tf.Exception,
                tf.ConnectivityException, tf.ExtrapolationException):
            return
#        print (trans,rot)

        target_pose = self.listener.transformPose( '/ekf/robot/base_stabilized',
                                                   relpose_cached )
        target_pose.header.stamp = rospy.Time.now()
        target_pose.header.seq = self.seq

        # target_pose.pose.orientation is the orientation of the 'trans',
        # which gives me a direction from where the robot is now to the beacon
        # not from where the robot should be to the beacon
        trans = ( -trans[0], -trans[1], -trans[2] )
        point = Point( *trans )

#        print point
        target_pose.pose.orientation = self.point_to_quaternion( point )

        self.nav_target.publish( target_pose )

    def run( self ):
        while not rospy.is_shutdown():
            self.seq += 1

            self.publish_target_pose()
            self.rate.sleep()

def main():
    rospy.init_node('falkor_quadrotor_navigate')
    navigator = FalkorQuadrotorNav()
    navigator.run()

if __name__  == '__main__':
    main()

