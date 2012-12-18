#!/usr/bin/env python
import roslib
roslib.load_manifest('falkor_quadrotor_control')

import rospy
import pid
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from std_srvs.srv import *
import tf

class TakeoffLand:
    def __init__( self ):
        self.ground_height = rospy.get_param( "~ground_height", 0.15 )
        self.takeoff_height = rospy.get_param( "~takeoff_height", 2.0 )
        self.takeoff_tol = rospy.get_param( "~takeoff_tol", 0.25 )
        self.land_tol = rospy.get_param( "~land_tol", 0.05 )
        self.listener = tf.TransformListener()

        self.height = None

        rospy.wait_for_service( "on" )
        self.on_service = rospy.ServiceProxy( "on", Empty )

        rospy.wait_for_service( "off" )
        self.off_service = rospy.ServiceProxy( "off", Empty )

        self.takeoff_service = rospy.Service( "takeoff", Empty, self.takeoff )
        self.land_service = rospy.Service( "land", Empty, self.land )

        self.cmd_vel_pub = rospy.Publisher( 'cmd_vel', Twist )

        self.sonar_sub = rospy.Subscriber( "sonar_height", Range, self.sonar_cb, queue_size=1 )
        self.busy = False
        self.on = False
        self.off = True

    def sonar_cb( self, data ):
        if data.range > data.max_range:
            self.height = data.max_range
        else:
            if data.range < data.min_range:
                data.range = data.min_range

            try:
                transform = self.listener.lookupTransform( '/robot/base_stabilized', '/robot/sonar_link', rospy.Time(0) )
                transform_matrix = tf.transformations.quaternion_matrix( transform[1] )
                point = transform_matrix[:3,:3].dot( np.array( [ data.range, 0, 0 ] ) )
            except (tf.LookupException, tf.Exception,
                    tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn( "takeoffland: transform exception: %s", str( e ) )
                self.height = data.range
            else:
                self.height = -point[2]

    def control( self, target_height, height_tol, pid_params ):
        my_pid = pid.PidController( *pid_params )
        my_pid.setPoint = target_height
        rate = rospy.Rate( 10 )
        last_time = rospy.Time.now()
        been_good = 0
        while been_good < 100:
            rate.sleep()
            time = rospy.Time.now()
            dt = ( last_time - time ).to_sec()
            cmd_vel = Twist()
            cmd_vel.linear.z = my_pid.get_output( self.height, dt )
            self.cmd_vel_pub.publish( cmd_vel )

            if abs( self.height - target_height ) < height_tol:
                been_good += 1
            else:
                been_good = 0
        
        # send a hover command
        self.cmd_vel_pub.publish( Twist() )

    def takeoff( self, req ):
        # change this so that if busy return a fail
        if self.busy or self.on:
            return EmptyResponse()
    
        self.busy = True
        self.on_service()
        self.control( self.takeoff_height, self.takeoff_tol, ( 2.0, 0, 0.1 ) )
        self.on = True
        self.busy = False

        return EmptyResponse()

    def land( self, req ):
        # change this so that if busy return a fail
        if self.busy or not self.on:
            return EmptyResponse()

        self.busy = True
        self.control( self.ground_height, self.land_tol, ( 0.5, 0, 0.1 ) )
        self.off_service()
        self.on = False
        self.busy = False
        return EmptyResponse()
     

    def run( self ):
        rospy.spin()
        
def main():
    rospy.init_node('takeoff_land')
    control = TakeoffLand()

    try:
        control.run()
    except ( KeyboardInterrupt, rospy.Exceptions.ROSInterruptException ) as e:
        print "Shutting down"

if __name__  == '__main__':
    main()

