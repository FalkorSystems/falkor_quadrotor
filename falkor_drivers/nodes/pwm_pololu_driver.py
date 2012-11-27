#!/usr/bin/env python

import roslib; roslib.load_manifest("falkor_drivers")
import rospy
import struct
from serial import *
from falkor_msgs.msg import *

## {{{ http://code.activestate.com/recipes/496969/ (r1)
#convert string to hex
def toHex(s):
    lst = []
    for ch in s:
        hv = hex(ord(ch)).replace('0x', '')
        if len(hv) == 1:
            hv = '0'+hv
        lst.append(hv)
    
    return reduce(lambda x,y:x+y, lst)

#convert hex repr to string
def toStr(s):
    return s and chr(atoi(s[:2], base=16)) + toStr(s[2:]) or ''
## end of http://code.activestate.com/recipes/496969/ }}}

class Pololu:
    def __init__(self, device_id, port):
        self.device_id = device_id
        self.port = port

        # send detect baud rate command
        cmd = '\xAA'
        self.port.write(cmd)

    ## Compact protocol
    def set_pos(self, servo_id, microseconds):
        position_value = int( microseconds * 4 )
        data1 = ( position_value >> 7 ) & 0x7F
        data2 = position_value & 0x7F
        command_byte = 0x84

        data2 = ( position_value >> 7 ) & 0x7F
        data1 = position_value & 0x7F
        msg = struct.pack( 'Bbbb', command_byte, servo_id, data1, data2 )
        self.port.write( msg )

class PwmOutputDriver:
    def __init__( self, port ):
        self.pwm_topic = rospy.get_param( "~pwm_out_topic", "pwm_out" )
        self.device_id = rospy.get_param( "~pololu_id", 0x01 )

        self.port = port
        self.pololu = Pololu( self.device_id, self.port )

        self.pwm_sub = rospy.Subscriber( self.pwm_topic, Pwm, self.pwm_cb )
        self.device_id = 0x01;

    def pwm_cb(self, data):
        for i in range(0,len(data.pwm)):
            rospy.logwarn( "sending %d to %d" % ( data.pwm[i], i ) )
            self.pololu.set_pos( i, data.pwm[i] )

    def run(self):
        rospy.spin()

def main():
    rospy.init_node('pwm_pololu_driver')

    port_name = rospy.get_param( "~port", "/dev/ttyUSB1" )
    baud = rospy.get_param( "~baud", 115200 )
    timeout = rospy.get_param( "~timeout", 5.0 )

    port = Serial(port_name, baud, timeout=timeout * 0.5)

    driver = PwmOutputDriver(port)
    try:
        driver.run()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__  == '__main__':
    main()
