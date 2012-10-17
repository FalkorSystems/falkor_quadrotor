#!/usr/bin/env python

import roslib
roslib.load_manifest('falkor_quadrotor_navigate')

import rospy
import numpy as np
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

        self.sonar_variance = rospy.get_param( '~sonar_variance', 0.5 )
        self.sonar_max_variance = rospy.get_param( '~sonar_max_variance', 1000 )
        self.num_particles = rospy.get_param( '~num_particles', 100 )
        self.particles_initialized = False
        self.last_time = None
        self.robot_state = None
        self.sonar_dist = None
        self.beacon_state = None
        self.rate = rospy.Rate( rospy.get_param( '~update_rate', 10 ) )
        self.seq = 0

    def initialize_particles( self ):
        if self.beacon_state == None:
            return

        # create a bunch of particles, picking from a gaussian centered around beacon_state
        self.particles = np.random.multivariate_normal( self.beacon_state[0],
                                                        self.beacon_state[1],
                                                        self.num_particles )
        self.particles_initialized = True
        self.publish_particles()

    def gaussian_pdf( self, x, m, covariance ):
        cov_matrix = np.matrix( covariance )
        x_matrix = np.matrix( x ).T
        m_matrix = np.matrix( m ).T

        det = np.linalg.det( cov_matrix )
        exp = np.exp( -0.5 * (x_matrix-m_matrix).T * cov_matrix.I * ( x_matrix-m_matrix ) )
        pdf = 1/np.sqrt( np.power( 2 * np.pi, x_matrix.size ) * det ) * exp
        return pdf[0,0]

    def particle_weight( self, particle ):
        # probability given the beacon's GPS/IMU data
        prob_beacon_data = self.gaussian_pdf( particle,
                                              self.beacon_state[0],
                                              self.beacon_state[1] )


        # probability values given the robot GPS/IMU plus distance data
        # move point by distance towards the center of the robot GPS/IMU
        vector_to_robot = self.robot_state[0] - particle
        distance_to_robot = np.linalg.norm( vector_to_robot )

        normalized_vector_to_robot = vector_to_robot / distance_to_robot

        # move the particle towards the robot by a random amount
        particle_moved = particle + ( normalized_vector_to_robot * 
                                      np.random.normal( self.sonar_dist[0],
                                                        np.sqrt( self.sonar_dist[1] ) ) )

        prob_robot_distance_data = self.gaussian_pdf( particle_moved,
                                                      self.robot_state[0],
                                                      self.robot_state[1] )

        weight = prob_beacon_data * prob_robot_distance_data
#        import pdb; pdb.set_trace()
        return weight

    def resample_particles( self ):
        weights = np.float64( [ self.particle_weight( particle ) for particle in self.particles ] )

        weights_normalized = weights / np.sum( weights )
        self.publish_particles()
#        import pdb; pdb.set_trace()

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

#    def save_data( self, best_guess, covariance ):#        data = np.float64( ( self.beacon_state, self.robot_state, self.sonar_dist, best_guess, covariance ) )
#        numpy.savetext( "output.txt", data, delimiter="," )

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

        
