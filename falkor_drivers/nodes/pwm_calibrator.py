#!/usr/bin/env python
import roslib; roslib.load_manifest( 'falkor_drivers' )

import rospy
from falkor_msgs.msg import *

pwm_values = None
pwm_max = None
pwm_min = None
pwm_calibration = { 'center': [],
                    'range': [],
                    'min': [] }

def pwm_in_cb( data ):
    pwm_values = data.pwm

    if pwm_max == None:
        pwm_max = pwm_values
    else:
        for i in range( 0, len( pwm_max ) ):
            if pwm_max[i] < pwm_values[i]:
                pwm_max[i] = pwm_values[i]:

    if pwm_min == None:
        pwm_min = pwm_values:
    else:
        for i in range( 0, len( pwm_min ) ):
            if pwm_min[i] > pwm_values[i]:
                pwm_min[i] = pwm_values[i]

rospy.init_node( 'pwm_calibrator' )
sub = rospy.Subscriber( "pwm_in", Pwm, pwm_in_cb )

# centering step
print "Make sure all sticks are centered.. waiting 5 seconds)"
rospy.sleep(5)

for value in pwm_values:
    pwm_calibration['center'].append( value )

print "Calibration:"
print pwm_calibration

print "OK now move all sticks to the extremes.. waiting 60 seconds"
rospy.sleep(60)

for (min,max) in zip(pwm_min,pwm_max)
    pwm_calibration['min'].append( min )
    pwm_calibration['range'].append( max - min )

print "Now sending PWM calibration signals"
pwm_min = rospy.get_param( "~pwm_min", 1000 )
pwm_max = rospy.get_param( "~pwm_max", 2000 )
pwm_center = ( pwm_min + pwm_max ) / 2

pub = rospy.Publisher( "pwm_out", Pwm )

pwm_msg = Pwm( [ pwm_min ] * 4 )
pub.publish( pwm_msg )
rospy.sleep( 1 )

pwm_msg = Pwm( [ pwm_max ] * 4 )
pub.publish( pwm_msg )
rospy.sleep( 1 )

pwm_msg = Pwm( [ pwm_center] * 4 )
pub.publish( pwm_msg )


    
