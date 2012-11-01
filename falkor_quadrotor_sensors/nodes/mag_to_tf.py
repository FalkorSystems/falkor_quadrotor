#!/usr/bin/env python
import roslib
roslib.load_manifest( 'falkor_quadrotor_sensors' )

import rospy
import tf
import numpy as np
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from nav_msgs.msg import *

class FalkorMagToTf:
    def __init__( self ):
        self.reference_heading = rospy.get_param( "~referenceHeading", 0 )
        self.declination = rospy.get_param( "~declination", -13.083 )
        self.world_frame = rospy.get_param( "~world_frame", '/nav' )
        self.magnetic_field_frame = rospy.get_param( "~magnetic_field_frame", self.world_frame + "/magnet" )

        self.ref_head_rad = self.to_rad( self.reference_heading )
        self.decl_rad = self.to_rad( self.declination )

        self.tf = tf.TransformBroadcaster()
        self.magnetic_field_world = tf.transformations.quaternion_from_euler( 0,
                                                                              0,
                                                                              self.reference_heading-self.decl_rad )
        self.update_rate = rospy.get_param( "~update_rate", 10 )
        self.rate = rospy.Rate( self.update_rate )

    def to_rad( self, degrees ):
        return degrees / 180.0 * np.pi

    def send_transform( self ):
        # send a transform for two timesteps from now
        self.tf.sendTransform( (0.0, 0.0, 0.0),
                               self.magnetic_field_world,
                               rospy.Time.now() + rospy.Duration( 2/self.update_rate ),
                               self.magnetic_field_frame,
                               self.world_frame )

    def run( self ):
        while not rospy.is_shutdown():
            self.send_transform()
            self.rate.sleep()

def main():
    rospy.init_node('mag_to_tf')
    tf_broadcaster = FalkorMagToTf()
    try:
        tf_broadcaster.run()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__  == '__main__':
    main()
            
        
        
