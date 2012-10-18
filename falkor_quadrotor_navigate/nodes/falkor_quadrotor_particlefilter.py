#!/usr/bin/env python

import roslib
roslib.load_manifest('falkor_quadrotor_navigate')

import rospy
import tf
import numpy as np
import scipy.stats as stats
from geometry_msgs.msg import *
from nav_msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import *

class FalkorQuadrotorParticleFilter:
    def filter_odom_msg( self, odom_msg ):
        # we only care about point and relevant covariance
        point = odom_msg.pose.pose.position
        point_np = np.float64( ( point.x, point.y, point.z ) )

        pt_covariance_matrix = np.float64( odom_msg.pose.covariance ).reshape( ( 6, 6 ) )
        pt_covariance_mat_limited = pt_covariance_matrix[:3,:3]

        vel = odom_msg.twist.twist.linear
        vel_np = np.float64( ( vel.x, vel.y, vel.z ) )

        vel_covariance_matrix = np.float64( odom_msg.twist.covariance ).reshape( ( 6, 6 ) )
        vel_covariance_mat_limited = vel_covariance_matrix[:3,:3]

        return( point_np, pt_covariance_mat_limited, vel_np, vel_covariance_mat_limited )

    def beacon_state_cb( self, data ):
        self.beacon_state = self.filter_odom_msg( data )

    def robot_state_cb( self, data ):
        self.robot_state = self.filter_odom_msg( data )

    def sonar_dist_cb( self, data ):
        # if we're at min/max then the data is invalid
        if data.range <= data.min_range or data.range >= data.max_range:
            self.sonar_dist = ( -1, self.sonar_max_variance )
        else:
            self.sonar_dist = ( data.range, self.sonar_variance )
            
    def __init__( self ):
        self.beacon_state_sub = rospy.Subscriber( '/beacon/state', Odometry,
                                                  self.beacon_state_cb )
        self.robot_state_sub = rospy.Subscriber( '/robot/state', Odometry,
                                                 self.robot_state_cb )
        self.sonar_dist_sub = rospy.Subscriber( '/beacon/sonar', Range,
                                                self.sonar_dist_cb )
        self.pose_pub = rospy.Publisher( '/beacon/pf/pose', PoseWithCovarianceStamped )
        self.pointcloud_pub = rospy.Publisher( '/beacon/pf/point_cloud', PointCloud )
        self.tf_broadcaster = tf.TransformBroadcaster()

        self.sonar_variance = rospy.get_param( '~sonar_variance', 0.5 )
        self.sonar_max_variance = rospy.get_param( '~sonar_max_variance', 10000 )
        self.num_particles = rospy.get_param( '~num_particles', 10000 )
        self.particles_initialized = False
        self.last_time = None
        self.robot_state = None
        self.sonar_dist = None
        self.beacon_state = None
        self.rate = rospy.Rate( rospy.get_param( '~update_rate', 10 ) )
        self.seq = 0

    def initialize_particles( self ):
        if self.robot_state == None or self.sonar_dist == None:
            return

        # create a bunch of particles, picking from a gaussian centered around robot_state
        self.particles = np.random.multivariate_normal( self.robot_state[0],
                                                        self.robot_state[1],
                                                        self.num_particles )

        # Now move the particles out in a random direction (uniformly distributed)
        # determined by the sonar distance
        distances = np.random.normal( self.sonar_dist[0], np.sqrt( self.sonar_dist[1] ),
                                      self.num_particles )
        directions_polar = np.random.rand( self.num_particles, 2 ) * [ np.pi, np.pi*2 ]

        directions_cartesian = np.array( ( np.sin( directions_polar[:,0] ) * np.cos( directions_polar[:,1] ),
                                           np.sin( directions_polar[:,0] ) * np.sin( directions_polar[:,1] ),
                                           np.cos( directions_polar[:,0] ) ) )

        self.particles += ( distances * directions_cartesian ).transpose()

        self.particles_initialized = True
        self.publish_particles()
        
    def mv_norm_pdf( values, mean, Sigma ):
        det = np.linalg.det( Sigma )
        k = mean.size
        n = values.shape[0]
        if not k == values.shape[1]:
            raise Exception( "shape of values doesn't match shape of mean" )
    
        values_centered = values - mean
        Sigma_inv = np.linalg.inv( Sigma )

        exp = np.exp( -0.5 * np.diag( values_centered.dot( Sigma ).dot( values_centered.transpose() ) ) )

        pdf = ( np.power( 2 * np.pi, - k / 2 ) * np.power( det, -1 / 2 ) *
                exp )
        return pdf

    def particle_weights( self ):
        # probability values given the sonar data
        vector_to_robot = self.robot_state[0] - self.particles
        distance_to_robot = np.sqrt( np.square( vector_to_robot[:,0] ) +
                                     np.square( vector_to_robot[:,1] ) +
                                     np.square( vector_to_robot[:,2] ) )
        distance_dist = stats.norm( self.sonar_dist[0], np.sqrt( self.sonar_dist[1] ) )
        prob_robot_distance_data = distance_dist.pdf( distance_to_robot )

        # probability values given the beacon data

        # probability values given the robot data

        weights = prob_robot_distance_data
        return weights

    def resample_particles( self ):
        weights = self.particle_weights()

        weights_normalized = weights / np.sum( weights )
        self.publish_particles()

        particles_cdf = np.cumsum( weights_normalized )
        randoms = np.random.sample( self.num_particles )
        self.particles = self.particles[np.searchsorted( particles_cdf, randoms )]

        best_guess = np.mean( self.particles, 0 )
        covariance = np.cov( self.particles, rowvar = 0 )
        return best_guess, covariance

    def move_particles( self, dt ):
        movements = np.random.multivariate_normal( self.beacon_state[2],
                                                   self.beacon_state[3],
                                                   self.num_particles ) * dt

        self.particles += movements
        self.publish_particles()

    def publish_particles( self ):
        pointcloud_msg = PointCloud()
        pointcloud_msg.header = Header( self.seq, rospy.Time.now(), '/nav' )
        for particle in self.particles:
            pointcloud_msg.points.append( Point32( *particle ) )

        self.pointcloud_pub.publish( pointcloud_msg )

    def publish_pose( self, best_guess, covariance ):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = Header( self.seq, rospy.Time.now(), "/nav" )
        pose_msg.pose.pose.position = Point( *best_guess )
        pose_msg.pose.pose.orientation = Quaternion( 0, 0, 0, 1 )
        covariance_flat = covariance.reshape( (1,9) )
#        pose_msg.pose.covariance = np.asarray( covariance_flat ) + [ 1e-10, 0, 0, 0, 1e-10, 0, 0, 0, 1e-10 ]

        self.pose_pub.publish( pose_msg )

        self.tf_broadcaster.sendTransform( ( best_guess[0], best_guess[1], 0.0 ),
                                           ( 0.0, 0.0, 0.0, 1.0 ),
                                           rospy.Time.now(),
                                           "/pf/beacon/base_footprint",
                                           "/nav" )

        self.tf_broadcaster.sendTransform( ( 0.0, 0.0, best_guess[2] ),
                                           ( 0.0, 0.0, 0.0, 1.0 ),
                                           rospy.Time.now(),
                                           "/pf/beacon/base_position",
                                           "/pf/beacon/base_footprint" )

    def update_pf( self ):
        # initialize the particles with data we have
        if not self.particles_initialized:
            self.last_time = rospy.Time.now()
            self.initialize_particles()
        else:
            # run the filter with the data we have
            time_now = rospy.Time.now()

            dt = ( time_now - self.last_time ).to_sec()
            self.last_time = time_now

            self.move_particles( dt )
            best_guess, covariance = self.resample_particles()

#            self.save_data( best_guess, covariance )
            self.publish_pose( best_guess, covariance )

    def run( self ):
        while not rospy.is_shutdown():
            self.seq += 1
            self.update_pf()
            self.rate.sleep()

def main():
    rospy.init_node('falkor_quadrotor_particlefilter')
    filt = FalkorQuadrotorParticleFilter()
    filt.run()

if __name__  == '__main__':
    main()

        
