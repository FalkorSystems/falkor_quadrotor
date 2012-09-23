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
    def update_relative_pose( self, data ):
        self.relative_pose = PoseStamped()
        self.relative_pose.pose.position = data.point
        self.relative_pose.header = data.header

        bearing = numpy.arctan2( data.point.y, -data.point.x )
        inclination = numpy.arctan2( data.point.z, numpy.sqrt( data.point.x*data.point.x +
                                                               data.point.y*data.point.y ) )
        self.relative_pose.pose.orientation = Quaternion( *tuple( tf.transformations.quaternion_from_euler( 0.0, inclination, -bearing ) ) )

    def __init__( self ):
        self.seq = 0
        self.listener = tf.TransformListener()

        self.subscriber = rospy.Subscriber( 'beacon/target_point', PointStamped,
                                            self.update_relative_pose )
        self.nav_target = rospy.Publisher( 'robot/target_pose', PoseStamped )
        relative_point = PointStamped( Header( 0, rospy.Time.now(),
                                              '/beacon/estimate/base_position' ),
                                      Vector3( 1, 0.5, 0.25 ) )

        self.update_relative_pose( relative_point )
        self.publish_target_pose()

        self.rate = rospy.Rate(10.0)


    def publish_target_pose( self ):
        try:
            print "Waiting for transform for " + str( self.relative_pose.header.stamp )
            self.listener.waitForTransform( '/robot/estimate/base_stabilized',
                                            '/beacon/estimate/base_position', self.relative_pose.header.stamp,
                                            rospy.Duration( 4.0 ) )

            (trans,rot) = self.listener.lookupTransform('/robot/estimate/base_stabilized',
                                                        '/beacon/estimate/base_position',
                                                        self.relative_pose.header.stamp )
        except (tf.LookupException, tf.Exception,
                tf.ConnectivityException, tf.ExtrapolationException):
            print "waiting Failed"
            return
        print (trans,rot)

        target_pose = self.listener.transformPose( '/robot/estimate/base_stabilized', self.relative_pose )
        target_pose.header.stamp = rospy.Time.now()
        target_pose.header.seq = self.seq

        print target_pose
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

