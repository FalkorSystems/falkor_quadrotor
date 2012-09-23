#!/usr/bin/env python
#
# we listen to a Vector3Stamped
# which gives us a desired position that the quad should have relative to
# the beacon, this is going to be in base_position frame of the beacon
#
# Vector3Stamped is a vector where x = distance, y = bearing from North and z =
# inclination from level
# 
# We then transform the Vector3Stamped into the robot's base_stabilized
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
    def convert_to_bearing( self, point ):
        return( bearing, inclination )

    def update_relative_position( self, data ):
        self.relative_position = data
        bearing = -numpy.arctan2( - data.point.y, data.point.x )
        inclination = -numpy.arctan2( data.point.z, data.point.x )
        self.relative_quaternion = Quaternion( *tuple( tf.transformations.quaternion_from_euler( 0.0, inclination, bearing ) ) )

    def __init__( self ):
        self.listener = tf.TransformListener()

        self.subscriber = rospy.Subscriber( '/beacon/target_point', PointStamped,
                                            self.update_relative_position )
        self.nav_target = rospy.Publisher( '/robot/target_pose', PoseStamped )
        self.relative_position = PointStamped( Header( 0, rospy.Time.now(), '/beacon/estimate/base_position' ),
                                                 Vector3( 10, 10, 10 ) )
        self.relative_quaternion = Quaternion( *tuple( tf.transformations.quaternion_from_euler( 0.0, 0.0, 0.0 ) ) )

        self.rate = rospy.Rate(10.0)
        self.seq = 0

    def run( self ):
        while not rospy.is_shutdown():
            self.seq += 1

            if not self.relative_position:
                continue
    
            try:
                self.listener.waitForTransform( '/robot/estimate/base_stabilized',
                                                '/beacon/estimate/base_position', self.relative_position.header.stamp,
                                                rospy.Duration( 4.0 ) )

                (trans,rot) = self.listener.lookupTransform('/robot/estimate/base_stabilized',
                                                            '/beacon/estimate/base_position',
                                                            self.relative_position.header.stamp )
            except (tf.LookupException,
                    tf.ConnectivityException, tf.ExtrapolationException):
                continue

            target_point = self.listener.transformPoint( '/robot/estimate/base_stabilized', self.relative_position )
        
            # Calculate a quaternion from the relative_position
            relative_quaternion_stamped = QuaternionStamped( self.relative_position.header, self.relative_quaternion )

            target_quaternion = self.listener.transformQuaternion( '/robot/estimate/base_stabilized', relative_quaternion_stamped )
            
            target_pose = PoseStamped( target_point.header, Pose( target_point.point, target_quaternion.quaternion ) )
            target_pose.header.stamp = rospy.Time.now()
            target_pose.header.seq = self.seq

            self.nav_target.publish( target_pose )
            self.rate.sleep()

def main():
    rospy.init_node('falkor_quadrotor_navigate')
    navigator = FalkorQuadrotorNav()
    navigator.run()

if __name__  == '__main__':
    main()

