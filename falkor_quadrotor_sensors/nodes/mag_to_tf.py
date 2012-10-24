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
        self.inclination = rospy.get_param( "~inclination", 66.867 )
        self.world_frame = rospy.get_param( "~world_frame", '/nav' )
        self.magnetic_field_frame = rospy.get_param( "~magnetic_field_frame", self.world_frame + "/magnet" )

        self.ref_head_rad = self.to_rad( self.reference_heading )
        self.decl_rad = self.to_rad( self.declination )
        self.incl_rad = self.to_rad( self.inclination )

        self.tf = tf.TransformBroadcaster()
        self.magnet_sub = rospy.Subscriber( "magnetic", Vector3Stamped,
                                            self.magnet_cb )

        self.magnetic_field_world = tf.transformations.quaternion_from_euler( 0,
                                                                              self.incl_rad,
                                                                              -self.decl_rad )

    def to_rad( self, degrees ):
        return degrees / 180.0 * np.pi

    def magnet_cb( self, data ):
        magnetometer_frame = data.header.frame_id
        yaw = np.arctan2( - data.vector.y, data.vector.x ) 
        z = data.vector.z / np.sqrt( np.square( data.vector.y ) +
                                     np.square( data.vector.x ) +
                                     np.square( data.vector.z ) )
        pitch_raw = np.arcsin( z )

        pitch = np.cos( yaw ) * pitch_raw
        roll = np.sin( yaw ) * pitch_raw

        self.tf.sendTransform( (0.0, 0.0, 0.0),
                               self.magnetic_field_world,
                               rospy.Time.now(),
                               self.magnetic_field_frame,
                               self.world_frame )

        self.tf.sendTransform( (0.0, 0.0, 0.0),
                               tf.transformations.quaternion_from_euler( roll, pitch, yaw ),
                               rospy.Time.now(),
                               magnetometer_frame,
                               self.magnetic_field_frame )

    def run( self ):
        rospy.spin()

def main():
    rospy.init_node('mag_to_tf')
    tf_broadcaster = FalkorMagToTf()
    try:
        tf_broadcaster.run()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__  == '__main__':
    main()
            
        
        
