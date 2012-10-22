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

class GpsModel:
    d_stddev = 0.1
    eps_stddev = 0.1
    f = 1/3600
    d = 5.0

class Publisher:
    def __init__( self, prefix, tf_prefix, world_frame, tf_broadcaster = tf.TransformBroadcaster() ):
        self.tf_prefix = tf_prefix
        self.world_frame = world_frame
        self.pose_pub = rospy.Publisher( prefix + '/pose', PoseWithCovarianceStamped )
        self.state_pub = rospy.Publisher( prefix + '/state', Odometry )
        self.tf_broadcaster = tf_broadcaster
        self.seq = 0

    def publish( self, best_guess_p, covariance_p,
                       best_guess_v, covariance_v
                 ):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = Header( self.seq, rospy.Time.now(), '/nav' )
        pose_msg.pose.pose.position = Point( *best_guess_p )
        pose_msg.pose.pose.orientation = Quaternion( 0, 0, 0, 1 )
        pose_msg.pose.covariance = covariance_p.reshape( (1,9) )

        self.pose_pub.publish( pose_msg )

        odom_msg = Odometry()
        odom_msg.header = pose_msg.header
        odom_msg.pose = pose_msg.pose
        odom_msg.twist.twist.linear = Vector3( *best_guess_v )
        odom_msg.twist.covariance = covariance_v.reshape( ( 1,9 ) )

        self.state_pub.publish( odom_msg )

        self.tf_broadcaster.sendTransform( ( best_guess_p[0], best_guess_p[1], best_guess_p[2] ),
                                           ( 0.0, 0.0, 0.0, 1.0 ),
                                           rospy.Time.now(),
                                           self.tf_prefix + "/base_position",
                                           self.world_frame )

        self.tf_broadcaster.sendTransform( ( best_guess_p[0], best_guess_p[1], 0.0 ),
                                           ( 0.0, 0.0, 0.0, 1.0 ),
                                           rospy.Time.now(),
                                           self.tf_prefix + "/base_footprint",
                                           self.world_frame )

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
            self.sonar_dist = ( data.range, self.sonar_stddev )
            
    def __init__( self ):
#        self.pointcloud_pub = rospy.Publisher( '/beacon/pf/point_cloud_rel', PointCloud )
        self.pointcloud_pub_beacon = rospy.Publisher( '/beacon/pf/point_cloud', PointCloud )
        self.pointcloud_pub_robot = rospy.Publisher( '/robot/pf/point_cloud', PointCloud )
        self.tf_broadcaster = tf.TransformBroadcaster()

        self.sonar_stddev = rospy.get_param( '~sonar_stddev', 0.5 )
        self.num_particles = rospy.get_param( '~num_particles', 100 )
        self.world_frame = rospy.get_param( '~world_frame', '/nav' )

        self.particles_initialized = False
        self.last_time = None
        self.robot_state = None
        self.sonar_dist = None
        self.beacon_state = None
        self.rate = rospy.Rate( rospy.get_param( '~update_rate', 10 ) )
        self.seq = 0

        self.beacon_state_sub = rospy.Subscriber( '/beacon/state', Odometry,
                                                  self.beacon_state_cb )
        self.robot_state_sub = rospy.Subscriber( '/robot/state', Odometry,
                                                 self.robot_state_cb )
        self.sonar_dist_sub = rospy.Subscriber( '/beacon/sonar', Range,
                                                self.sonar_dist_cb )

        self.publish_robot = Publisher( "/robot/pf", tf_prefix = "/pf/robot", world_frame = self.world_frame,
                                        tf_broadcaster = self.tf_broadcaster )
        self.publish_beacon = Publisher( "/beacon/pf", tf_prefix = "/pf/beacon", world_frame = self.world_frame,
                                        tf_broadcaster = self.tf_broadcaster )
    def append_arrays( self, arrays ):
        result = arrays[0]
        for p in arrays[1:]:
            result = np.append( result, p, axis=1 )

        return result

    def create_particles( self, num_particles ):
        # Now move the particles out in a random direction (uniformly distributed)
        # determined by the sonar distance

        # each particle is matrix
        # [ epsilon_beacon, x_beacon, d_beacon, v_beacon, epsilon_robot, x_robot, d_robot, v_robot ]
        epsilon_beacon = np.random.randn( num_particles, 3 ) * GpsModel.eps_stddev
        epsilon_robot = np.random.randn( num_particles, 3 ) * GpsModel.eps_stddev

        x_beacon = np.random.multivariate_normal( self.beacon_state[0],
                                                  self.beacon_state[1],
                                                  num_particles )

        x_robot = np.random.multivariate_normal( self.robot_state[0],
                                                 self.robot_state[1],
                                                 num_particles )

        d_robot = np.random.randn( num_particles, 3 ) * GpsModel.d_stddev
        d_beacon = np.random.randn( num_particles, 3 ) * GpsModel.d_stddev

        v_robot = np.random.multivariate_normal( self.robot_state[2],
                                                 self.robot_state[3],
                                                 num_particles )
        v_beacon = np.random.multivariate_normal( self.beacon_state[2],
                                                  self.beacon_state[3],
                                                  num_particles )
    
        particles = self.append_arrays( [ epsilon_beacon, x_beacon, d_beacon, v_beacon,
                                          epsilon_robot, x_robot, d_robot, v_robot ] )

        return particles

    def initialize_particles( self ):
        if self.robot_state == None or self.sonar_dist == None or self.beacon_state == None:
            return

        self.particles = self.create_particles( self.num_particles )
        self.weights = np.ones( self.num_particles ) / self.num_particles
        self.particles_initialized = True
        self.publish_particles()

    def reinitialize_particles( self ):
        return
        portion = 1000
        new_particles = self.create_particles( int( self.num_particles / portion ) )
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

    def vector_probs( self, mean_vector, stddev_vector, data_vector ):
        dist = [ stats.norm( mean_vector[0], stddev_vector[0] ),
                 stats.norm( mean_vector[1], stddev_vector[1] ),
                 stats.norm( mean_vector[2], stddev_vector[2] ) ]

        prob = ( dist[0].pdf( data_vector[:,0] ) *
                 dist[1].pdf( data_vector[:,1] ) *
                 dist[2].pdf( data_vector[:,2] ) )
        return prob

    def distance( self, vector ):
        return np.sqrt( np.square( vector[:,0] ) +
                        np.square( vector[:,1] ) +
                        np.square( vector[:,2] ) )
    

    def compute_particle_weights( self ):
        distance_to_beacon = self.distance( self.get_x_beacon() - self.get_x_robot() )

        distance_dist = stats.norm( self.sonar_dist[0], self.sonar_dist[1] )
        prob_distance = distance_dist.pdf( distance_to_beacon )

        v_robot_probs = self.vector_probs( self.robot_state[2], [ GpsModel.eps_stddev,
                                                                  GpsModel.eps_stddev,
                                                                  GpsModel.eps_stddev ],
                                           self.get_v_robot() )
        v_beacon_probs = self.vector_probs( self.beacon_state[2], [ GpsModel.eps_stddev,
                                                                    GpsModel.eps_stddev,
                                                                    GpsModel.eps_stddev ],
                                           self.get_v_beacon() )

        d_robot_probs = self.vector_probs( [ 0, 0, 0 ],
                                           [ GpsModel.d_stddev, GpsModel.d_stddev, GpsModel.d_stddev ],
                                           self.robot_state[0] - ( self.get_x_robot() + self.get_d_robot() ) )

        d_beacon_probs = self.vector_probs( [ 0, 0, 0 ],
                                            [ GpsModel.d_stddev, GpsModel.d_stddev, GpsModel.d_stddev ],
                                            self.beacon_state[0] - ( self.get_x_beacon() + self.get_d_beacon() ) )

        weights = np.ones( self.num_particles )
        for probs in [ prob_distance, v_robot_probs, v_beacon_probs, d_robot_probs, d_beacon_probs ]:
            # remove zeros
            if np.sum( probs ) > 0:
                weights *= probs

        if np.sum( weights ) == 0:
            return
        else:
            self.weights = weights / np.sum( weights )

    def resample_particles( self ):
        particles_cdf = np.cumsum( self.weights )
        randoms = np.random.rand( self.num_particles )
        self.particles = self.particles[np.searchsorted( particles_cdf, randoms )]
        self.weights = np.ones( self.num_particles )

    def best_guess( self, x, v ):
        best_guess_p = np.average( x, axis=0, weights=self.weights )
        best_guess_v = np.average( v, axis=0, weights=self.weights )

        covariance_p = np.cov( x * self.weights.reshape( self.num_particles, 1 ), rowvar = 0 )
        covariance_v = np.cov( v * self.weights.reshape( self.num_particles, 1 ), rowvar = 0 )
        return ( best_guess_p, covariance_p,
                 best_guess_v, covariance_v )

    def get_d_beacon( self ):
        return self.particles[:,6:9]

    def get_d_robot( self ):
        return self.particles[:,18:21]

    def get_x_beacon( self ):
        return self.particles[:,3:6]

    def get_x_robot( self ):
        return self.particles[:,15:18]

    def get_v_beacon( self ):
        return self.particles[:,9:12]

    def get_v_robot( self ):
        return self.particles[:,21:24]

    def move_particles( self, dt ):
        epsilon_beacon = np.random.randn( self.num_particles, 3 ) * GpsModel.eps_stddev
        epsilon_robot = np.random.randn( self.num_particles, 3 ) * GpsModel.eps_stddev
        
        d_beacon = ( ( 1 - dt * GpsModel.f ) * self.get_d_beacon() +
                     ( epsilon_beacon -
                       np.random.randn( self.num_particles, 3 ) * GpsModel.d * np.sqrt( 2 * GpsModel.f ) )
                     * dt )
        d_robot = ( ( 1 - dt * GpsModel.f ) * self.get_d_robot() +
                    ( epsilon_robot -
                      np.random.randn( self.num_particles, 3 ) * GpsModel.d * np.sqrt( 2 * GpsModel.f ) )
                    * dt )

        v_robot = self.get_v_robot() + epsilon_robot
        v_beacon = self.get_v_beacon() + epsilon_beacon

        x_robot = self.get_x_robot() + self.get_v_robot() * dt
        x_beacon = self.get_x_beacon() + self.get_v_beacon() * dt

        self.particles = self.append_arrays( [ epsilon_beacon, x_beacon, d_beacon, v_beacon,
                                               epsilon_robot, x_robot, d_robot, v_robot ] )

        self.publish_particles()

    def publish_particles( self ):

#        pointcloud_msg.header = Header( self.seq, rospy.Time.now(), '/ground_truth/robot/base_position' )
        header = Header( self.seq, rospy.Time.now(), self.world_frame )
        channels = [ ChannelFloat32( 'intensity', [] ) ]

        for weight in self.weights:
            channels[0].values.append( weight )
        
        beacon_points = robot_points = []
        for pos in self.get_x_beacon():
            beacon_points.append( Point32( *pos ) )

        for pos in self.get_x_robot():
            robot_points.append( Point32( *pos ) )

        pointcloud_beacon = PointCloud( header, beacon_points, channels )
        pointcloud_robot = PointCloud( header, robot_points, channels )

        self.pointcloud_pub_beacon.publish( pointcloud_beacon )
        self.pointcloud_pub_robot.publish( pointcloud_robot )

    def update_pf( self ):
        # initialize the particles with data we have
        if not self.particles_initialized:
            self.last_resample = self.last_time = rospy.Time.now()
            self.initialize_particles()
        else:
            # run the filter with the data we have
            time_now = rospy.Time.now()

            dt = ( time_now - self.last_time ).to_sec()
            self.last_time = time_now

            self.move_particles( dt )
            self.compute_particle_weights()
            if ( self.last_resample - time_now ).to_sec() > 1:
                self.resample_particles()
                self.last_resample = time_now

            ( best_guess_p, covariance_p, 
              best_guess_v, covariance_v ) = self.best_guess( self.get_x_robot(),
                                                              self.get_v_robot() )

            self.publish_robot.publish( best_guess_p, covariance_p,
                                        best_guess_v, covariance_v )


            ( best_guess_p, covariance_p, 
              best_guess_v, covariance_v ) = self.best_guess( self.get_x_beacon(),
                                                              self.get_v_beacon() )

            self.publish_beacon.publish( best_guess_p, covariance_p,
                                         best_guess_v, covariance_v )

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

        
