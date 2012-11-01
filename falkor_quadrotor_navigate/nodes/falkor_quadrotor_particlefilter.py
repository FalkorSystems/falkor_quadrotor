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
            self.sonar_dist = ( -1, self.sonar_maxstddev )
        else:
            self.sonar_dist = ( data.range, self.sonar_stddev )
            
    def __init__( self ):
        self.pose_pub = rospy.Publisher( '/beacon/pf/pose', PoseWithCovarianceStamped )
        self.pointcloud_pub = rospy.Publisher( '/beacon/pf/point_cloud', PointCloud )
        self.tf_broadcaster = tf.TransformBroadcaster()

        self.sonar_stddev = rospy.get_param( '~sonar_stddev', 0.05 )
        self.sonar_maxstddev = rospy.get_param( '~sonar_maxstddev', 100 )
        self.num_particles = rospy.get_param( '~num_particles', 10000 )
        self.particles_initialized = False
        self.last_time = None
        self.robot_state = None
        self.sonar_dist = None
        self.beacon_state = None
        self.rate = rospy.Rate( rospy.get_param( '~update_rate', 10 ) )
        self.seq = 0

        self.beacon_state_sub = rospy.Subscriber( '/beacon/raw/state', Odometry,
                                                  self.beacon_state_cb )
        self.robot_state_sub = rospy.Subscriber( '/robot/raw/state', Odometry,
                                                 self.robot_state_cb )
        self.sonar_dist_sub = rospy.Subscriber( '/beacon/sonar', Range,
                                                self.sonar_dist_cb )

    def create_particles( self, num_particles ):
        # Now move the particles out in a random direction (uniformly distributed)
        # determined by the sonar distance
        distances = np.random.normal( self.sonar_dist[0], self.sonar_dist[1],
                                      num_particles )

        directions_z_theta = ( np.random.rand( num_particles, 2 ) - 0.5 ) * [ 2, np.pi*2 ]
        directions_cartesian = np.array( ( np.sin( directions_z_theta[:,1] ) * np.sqrt( 1 - np.square( directions_z_theta[:,0] ) ),
                                           np.cos( directions_z_theta[:,1] ) * np.sqrt( 1 - np.square( directions_z_theta[:,0] ) ),
                                           directions_z_theta[:,0] ) )

        # the particle vector is the relative distance [x,y,z] from the robot to the beacon
        particles = ( distances * directions_cartesian ).transpose()
        return particles
    
    def initialize_particles( self ):
        if self.robot_state == None or self.sonar_dist == None:
            return

        self.particles = self.create_particles( self.num_particles )
        self.particles_initialized = True
        self.weights = np.ones( self.num_particles ) / self.num_particles
        self.publish_particles()

    def reinitialize_particles( self ):
        portion = 1000
        num_new_particles = int( self.num_particles / portion )
        new_particles = self.create_particles( num_new_particles )
        self.particles[0:self.num_particles:portion] = new_particles
        
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

    def calculate_particle_weights( self ):
        distance_to_robot = np.sqrt( np.square( self.particles[:,0] ) +
                                     np.square( self.particles[:,1] ) +
                                     np.square( self.particles[:,2] ) )
        distance_dist = stats.norm( self.sonar_dist[0], self.sonar_dist[1] )
        prob_robot_distance_data = distance_dist.pdf( distance_to_robot )

        weights = prob_robot_distance_data

        self.weights = weights / np.sum( weights )

    def resample_particles( self ):
        particles_cdf = np.cumsum( self.weights )
        randoms = np.random.rand( self.num_particles )
        self.particles = self.particles[np.searchsorted( particles_cdf, randoms )]

    def get_best_guess( self ):
        # this guess is in the robot's frame
        best_guess = np.average( self.particles, axis=0, weights=self.weights )
        demeaned = self.particles - best_guess
        covariance = np.dot( self.weights * demeaned.T, demeaned )

        return best_guess, covariance

    def move_particles( self, dt ):
        movements_beacon = np.random.multivariate_normal( self.beacon_state[2],
                                                          self.beacon_state[3],
                                                          self.num_particles ) * dt
        movements_robot = np.random.multivariate_normal( self.robot_state[2],
                                                         self.robot_state[3],
                                                         self.num_particles ) * dt

        relative_movements = movements_beacon - movements_robot

        self.particles += relative_movements
        self.publish_particles()

    def publish_particles( self ):
        pointcloud_msg = PointCloud()
        pointcloud_msg.header = Header( self.seq, rospy.Time.now(), '/robot/base_position' )
        for particle in self.particles:
            # beacon - robot
            pointcloud_msg.points.append( Point32( *particle ) )

        self.pointcloud_pub.publish( pointcloud_msg )

    def publish_pose( self, best_guess, covariance ):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = Header( self.seq, rospy.Time.now(), "/robot/base_position" )
        pose_msg.pose.pose.position = Point( *best_guess )
        pose_msg.pose.pose.orientation = Quaternion( 0, 0, 0, 1 )
        pose_msg.pose.covariance = covariance.reshape( 9 )

        self.pose_pub.publish( pose_msg )


        self.tf_broadcaster.sendTransform( ( best_guess[0], best_guess[1], best_guess[2] ),
                                           ( 0.0, 0.0, 0.0, 1.0 ),
                                           rospy.Time.now(),
                                           "/beacon/pf/base_position",
                                           "/robot/base_position" )



        self.tf_broadcaster.sendTransform( ( 0.0, 0.0, -best_guess[2] ),
                                           ( 0.0, 0.0, 0.0, 1.0 ),
                                           rospy.Time.now(),
                                           "/beacon/pf/base_footprint",
                                           "/beacon/pf/base_position" )

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
            self.calculate_particle_weights()

            best_guess, covariance = self.get_best_guess()
            if self.seq % 60 == 0:
                self.resample_particles()

            self.publish_pose( best_guess, covariance )
            self.reinitialize_particles()

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

        
