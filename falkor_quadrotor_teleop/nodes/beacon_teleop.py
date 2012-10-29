#!/usr/bin/env python
import roslib
roslib.load_manifest('falkor_quadrotor_teleop')

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, SetModelStateRequest

class BeaconTeleopJoy:

    def __init__( self ):
        self.joy_sub = rospy.Subscriber( "joy", Joy, self.callback_joy )
        self.jump_button = rospy.get_param( "~jump_button", 1 )
        self.forward_button = rospy.get_param( "~forward_button", 2 )
        self.backward_button = rospy.get_param( "~backward_button", 3 )
        self.twist_forward_button = rospy.get_param( "~twist_forward_button", 4 )
        self.twist_backward_button = rospy.get_param( "~twist_backward_button", 5 )
        self.twist_left_button = rospy.get_param( "~twist_left_button", 4 )
        self.twist_right_button = rospy.get_param( "~twist_right_button", 5 )

        self.state_service = rospy.ServiceProxy( "/gazebo/set_model_state", SetModelState )
        self.model_states = rospy.Subscriber( "/gazebo/model_states", ModelStates, self.model_state_cb )
        self.my_model_state = None
        self.model_name = rospy.get_param( "~model_name", "beacon" )
        self.reference_frame = rospy.get_param( "~reference_frame", "world" )
        self.last_buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    def model_state_cb( self, data ):
       model_names = data.name
       try:
           model_ix = model_names.index( self.model_name )
           self.my_model_state = ModelState( self.model_name,
                                              data.pose[model_ix],
                                              data.twist[model_ix],
                                              self.reference_frame )
       except ValueError:
           self.my_model_state = None

    def set_model_state( self ):
        model_state_request = SetModelStateRequest( self.my_model_state )
        self.state_service( model_state_request )

    def twist_forward( self ):
        self.my_model_state.twist.angular.y += 1.0
        self.set_model_state()

    def twist_backward( self ):
        self.my_model_state.twist.angular.y -= 1.0
        self.set_model_state()

    def forward( self ):
        self.my_model_state.twist.linear.x += 1.0
        self.set_model_state()

    def backward( self ):
        self.my_model_state.twist.linear.x -= 1.0
        self.set_model_state()

    def twist_left( self ):
        self.my_model_state.twist.angular.x -= 1.0
        self.set_model_state()

    def twist_right( self ):
        self.my_model_state.twist.angular.x += 1.0
        self.set_model_state()

    def jump( self ):
        self.my_model_state.twist.linear.z += 10.0
        self.set_model_state()
        
    def callback_joy( self, data ):
        if data.buttons[self.jump_button] == 1 and self.last_buttons[self.jump_button] == 0:
            self.jump()

        if data.buttons[self.forward_button] == 1 and self.last_buttons[self.forward_button] == 0:
            self.forward()

        if data.buttons[self.backward_button] == 1 and self.last_buttons[self.backward_button] == 0:
            self.backward()

        if data.buttons[self.twist_forward_button] == 1 and self.last_buttons[self.twist_forward_button] == 0:
            self.twist_forward()

        if data.buttons[self.twist_backward_button] == 1 and self.last_buttons[self.twist_backward_button] == 0:
            self.twist_backward()

        if data.buttons[self.twist_left_button] == 1 and self.last_buttons[self.twist_left_button] == 0:
            self.twist_left()

        if data.buttons[self.twist_right_button] == 1 and self.last_buttons[self.twist_right_button] == 0:
            self.twist_right()

        self.last_buttons = data.buttons

def main():
  rospy.init_node( 'beacon_teleop_joy' )

  BeaconTeleopJoy()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"

if __name__ == '__main__':
    main()
            
    
