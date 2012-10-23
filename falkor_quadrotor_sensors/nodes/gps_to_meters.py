#!/usr/bin/env python
import roslib
roslib.load_manifest( 'falkor_quadrotor_sensors' )

import rospy
import numpy as np
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from nav_msgs.msg import *

class FalkorGpsToMeters:
    def __init__( self ):
        self.reference_heading = rospy.get_param( "~reference_heading", 0 )
        self.ref_head_rad = self.to_rad( self.reference_heading )

        self.rotation_matrix = np.array( [ [ np.cos( self.ref_head_rad ),  -np.sin( self.ref_head_rad ) ],
                                           [ -np.sin( self.ref_head_rad, ), np.cos( self.ref_head_rad ) ] ] )

        # RCSMP, Marine Park, Brooklyn
        # 40.589681,-73.91852
        self.reference_latitude = rospy.get_param( "~reference latitude", 40.589681 )
        self.ref_lat_rad = self.to_rad( self.reference_latitude )

        self.reference_longitude = rospy.get_param( "~reference_longitude", -73.918520 )
        self.ref_long_rad = self.to_rad( self.reference_longitude )

        # MSL
        self.reference_altitude = rospy.get_param( "~reference_altitude", 0 )

        self.earth_radius = 6371e3
        self.fix_topic = rospy.get_param( "~fix_topic", "fix" )
        self.vel_topic = rospy.get_param( "~fix_vel_topic", "fix_velocity" )

        self.fix_pub = rospy.Publisher( self.fix_topic + "/pose", PoseWithCovarianceStamped )
        self.vel_pub = rospy.Publisher( self.fix_topic + "/twist", TwistWithCovarianceStamped )
        self.state_pub = rospy.Publisher( self.fix_topic  + "/state", Odometry )

        self.world_frame = rospy.get_param( "~world_frame", "/nav" )

        self.fix_seq = 0
        self.vel_seq = 0

        # We this tells us how to rotate the covariance matrix
        scale_xy = self.xy( ( self.reference_latitude + 0.01, self.reference_longitude + 0.01 ) )
        self.scaled_rotation_matrix = np.array( [ [ scale_xy[0], scale_xy[1], 0 ],
                                                  [ scale_xy[1], scale_xy[0], 0 ],
                                                  [ 0, 0, 1.0 ] ] ) * 100

        self.scaled_rotation_matrix_inverse = np.linalg.inv( self.scaled_rotation_matrix )

        self.state_seq = 0
        self.state = Odometry()

        self.fix_sub = rospy.Subscriber( self.fix_topic, NavSatFix, self.fix_cb )

        # for now fix_velocity is a Vector3 but someday will want to start
        # using TwistWithCovarianceStamped
        self.vel_sub = rospy.Subscriber( self.vel_topic, Vector3Stamped, self.vel_cb )

    def to_rad( self, degrees ):
        return degrees / 180.0 * np.pi
    
    def xy( self, destination ):
        import pdb; pdb.set_trace()
        # From http://www.movable-type.co.uk/scripts/latlong.html
        lat1 = self.ref_lat_rad
        long1 = self.ref_long_rad
        lat2 = self.to_rad( destination[0] )
        long2 = self.to_rad( destination[1] )

        dlat = lat2-lat1
        dlong = long2-long1

        a = np.square( np.sin( dlat/2 ) ) + np.cos( lat1 ) * np.cos( lat2 ) * np.square( np.sin( dlong/2 ) )
        c = 2 * np.arctan2( np.sqrt( a ), np.sqrt( 1-a ) )
        distance = self.earth_radius * c

        y_tmp = np.sin( dlong ) * np.cos( lat2 )
        x_tmp = np.cos( lat1 ) * np.sin( lat2 ) - np.sin( lat1 ) * np.cos( lat2 ) * np.cos( dlong )
        bearing = np.arctan2( y_tmp, x_tmp )

        adjusted_bearing = ( bearing + self.ref_head_rad ) % ( 2 * np.pi )
        x = distance * np.cos( adjusted_bearing )
        y = - distance * np.sin( adjusted_bearing )
        return x, y

    def rotate_xy( self, x, y ):
        return self.rotation_matrix.dot( np.array( ( x, y ) ).transpose() )

    def publish_state( self, pose = None, vel = None ):
        self.state_seq += 1
        self.state.header = Header( self.state_seq, rospy.Time.now(), self.world_frame )
        if not pose == None:
            self.state.pose = pose.pose
        if not vel == None:
            self.state.twist = vel.twist

        self.state_pub.publish( self.state )

    def vel_cb( self, data ):
        x, y = self.rotate_xy( data.vector.x, data.vector.y )
        self.vel_seq += 1
        header = Header( self.vel_seq, rospy.Time.now(), self.world_frame )

        # for now covariance is zeros but we will have to take the input covariance and 
        # rotate it
        covariance = np.zeros( 36 )
        
        vel_msg = TwistWithCovarianceStamped( header, TwistWithCovariance( 
                Twist( Vector3( x, y, data.vector.z ), Vector3( 0, 0, 0 ) ),
                list( covariance ) ) )

        self.vel_pub.publish( vel_msg )
        self.publish_state( vel = vel_msg )

    def fix_cb( self, data ):
        if data.status.status == NavSatStatus.STATUS_NO_FIX:
            return
        x, y = self.xy( ( data.latitude, data.longitude ) )
        position = Point( x, y, data.altitude + self.reference_altitude )
        orientation = Quaternion( 0, 0, 0, 1 )

        if not data.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            # rotate, scale, and publish covariance
            covariance_matrix = np.array( data.position_covariance ).reshape( 3, 3 )
            new_covariance = self.scaled_rotation_matrix_inverse.dot( covariance_matrix ).dot( self.scaled_rotation_matrix )
        else:
            new_covariance = np.zeros( ( 3, 3 ) )

        # add zeros to make it 6x6
        new_covariance = np.append( new_covariance, np.zeros( ( 3, 3 ) ), 1 )
        new_covariance = np.append( new_covariance, np.zeros( ( 3, 6 ) ), 0 )

        covariance = list( new_covariance.reshape( 36 ) )

        pose = Pose( position, orientation )
        header = Header( self.fix_seq, rospy.Time.now(), self.world_frame )
        pose_msg = PoseWithCovarianceStamped( header, PoseWithCovariance( pose, covariance ) )
        self.fix_pub.publish( pose_msg )
        self.publish_state( pose = pose_msg )

    def run( self ):
        rospy.spin()

def main():
    rospy.init_node('gps_to_meters')
    converter = FalkorGpsToMeters()
    try:
        converter.run()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__  == '__main__':
    main()

            
        
