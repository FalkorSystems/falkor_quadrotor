#!/usr/bin/env python

import roslib; roslib.load_manifest("falkor_drivers")
import rospy
import struct
import falkor_msgs.msg
from mmap import mmap

#
# PWM control code comes from
# https://github.com/aquaticus/BeagleBone-tools.git
#

class Pwm:
    MMAP_OFFSET = 0x44c00000                # base address of registers
    MMAP_SIZE   = 0x48ffffff-MMAP_OFFSET    # size of the register memory space

    CM_PER_BASE = 0x44e00000 - MMAP_OFFSET
    CM_PER_EPWMSS1_CLKCTRL = CM_PER_BASE + 0xcc
    CM_PER_EPWMSS0_CLKCTRL = CM_PER_BASE + 0xd4
    CM_PER_EPWMSS2_CLKCTRL = CM_PER_BASE + 0xd8

    # PWM parameters
    # pin is the name of the pin for MODE 0, NOT BeagleBone signal name.
    pwm_map = {
        1 : {   'fs': '/sys/class/pwm/ehrpwm.1:0',
                'clk': CM_PER_EPWMSS1_CLKCTRL,
                'pin': 'gpmc_a2',
                'signal': 'EHRPWM1A', 
                'header': 'P9.14',
                'mode' : 6 
                },
        
        2 : {   'fs': '/sys/class/pwm/ehrpwm.1:1',
                'clk': CM_PER_EPWMSS1_CLKCTRL,
                'pin': 'gpmc_a3',
                'signal': 'EHRPWM1B', 
                'header': 'P9.16',
                'mode' : 6
                },

        3 : {   'fs': '/sys/class/pwm/ehrpwm.2:0',
                'clk': CM_PER_EPWMSS2_CLKCTRL,
                'pin': 'gpmc_ad8',
                'signal': 'EHRPWM2A', 
                'header': 'P8.19',
                'mode' : 4
                },

        4 : {   'fs': '/sys/class/pwm/ehrpwm.2:1',
                'clk': CM_PER_EPWMSS2_CLKCTRL,
                'pin': 'gpmc_ad9',
                'signal': 'EHRPWM2B', 
                'header': 'P8.13',
                'mode' : 4
                },

        5 : {   'fs': '/sys/class/pwm/ehrpwm.0:0',
                'clk': CM_PER_EPWMSS0_CLKCTRL,
                'pin': 'mcasp0_fsx',
                'signal': 'EHRPWM0A',
                'header': 'P9.31',
                'mode' : 1
                },

        6 : {   'fs': '/sys/class/pwm/ehrpwm.0:1',
                'clk': CM_PER_EPWMSS0_CLKCTRL,
                'pin': 'mcasp0_aclkx',
                'signal': 'EHRPWM0B',
                'header': 'P9.29',
                'mode' : 1
                },
        
        #Alternative #5 and #6 (but it's still ehrpwm0)
        50 : {  'fs': '/sys/class/pwm/ehrpwm.0:0',
                'clk': CM_PER_EPWMSS0_CLKCTRL,
                'pin': 'spi0_sclk',
                'signal': 'EHRPWM0A',
                'header': 'P9.22',
                'mode' : 3
                },

        60 : {  'fs': '/sys/class/pwm/ehrpwm.0:1',
                'clk': CM_PER_EPWMSS0_CLKCTRL,
                'pin': 'spi0_d0',
                'signal': 'EHRPWM0B',
                'header': 'P9.21',
                'mode' : 3            
                }
        }

    def printTable(self):
        """Prints table of supported PWM channels."""
        fmt = "%7s %-6s %-12s %-8s %s\n"
        table = "Supported BeagleBone PWM pins:\n"
        table += fmt % ('CHANNEL', 'HEADER', 'PIN', 'SIGNAL', 'FILE SYSTEM')

        for pwm in sorted(pwm_map.iterkeys()):
            p = Pwm.pwm_map[pwm]
            table += fmt % (pwm, p['header'], p['pin'], p['signal'], p['fs'])

        table += "PWM amplitude is 3.3V.\n" 
        table += "eCAP 2 additional PWM channels not supported.\n" 
        table += "PIN is MODE 0 name, NOT BeagleBone System Reference Manual signal name.\n" 

        return table

    def setReg(self, address, new_value):
        """ Sets 32 bits at given address to given value. """
        with open("/dev/mem", "r+b") as f:
            mem = mmap(f.fileno(), Pwm.MMAP_SIZE, offset=Pwm.MMAP_OFFSET)
            mem[address:address+4] = struct.pack("<L", new_value)

    def writeDevice(self, device, value):
        """ Write value to specified file. """
        with open(device,"w") as f:
            f.write("%d" % int(value))

    def setDutyPercent(self, device, dutyPercent):
        """ Set duty cycle in percents. """
        self.writeDevice("%s/duty_percent" % device, dutyPercent)

    def setDutyPercentByChannel(self, channel, dutyPercent):
        dev = Pwm.pwm_map[channel]['fs']
	print "setting channel %d to %d" % ( channel, dutyPercent )
        self.setDutyPercent(dev, dutyPercent)

    def getDutyPercent(self, device):
        """ Reads current duty cycle in percent. """
        with open("%s/duty_percent" % device,"r") as f:
            s = f.read()
        return int(s)

    def setFrequency(self, device, frequencyHz):
        """ Set PWM frequency. """
        self.writeDevice("%s/period_freq" % device, frequencyHz)

    def getFrequency(self, device):
        """ Reads current duty cycle in percent. """
        with open("%s/period_freq" % device,"r") as f:
            s = f.read()
        return int(s)

    def request(self, device, req):
        """ Request or release PWM channel. """
        self.writeDevice("%s/request" % device, req)
    
    def run(self, device, start):
        """ Start (1) or Stop (0) PWM. """
        with open("%s/run" % device,"r") as f: #read current state
            s = f.read()

        if start != int(s):
            self.writeDevice("%s/run" % device, start)

    def checkBusy(self, device):
        """ Check if device is busy. """
        with open("%s/request" % device,"r") as f:
            s = f.read()
            if s.find('is free') >= 0:
                return 0
            else:
                return 1

    def setMuxMode(self, pinName, mode):
        """ Set mux mode of the pin. """
        self.writeDevice("/sys/kernel/debug/omap_mux/%s" % pinName , mode)
    
    def __init__(self, channels, frequency):
        # Set up each channel
        for channel in channels:
            pwm = Pwm.pwm_map[channel]
            dev = pwm['fs']
            
            # setup clock
            self.setReg( pwm['clk'], 2 )

            if self.checkBusy(dev):
                raise Exception( "PWM channel %d (%s) is busy. You can try 'echo 0 > %s/request'" % (channel, dev, dev) )

            # set mux
            self.setMuxMode(pwm['pin'], pwm['mode'])

            # set frequency etc
            duty_percent = self.getDutyPercent(dev)
            self.run(dev, 0) #to change frequency pwm must be stopped
            self.setDutyPercent(dev, 0) #set to 0 before setting frequency
            self.setFrequency(dev, frequency) #set frequency
            self.setDutyPercent(dev, duty_percent) #back to previous duty cycle
            self.run(dev,1) #start PWM

class PwmDriver:
    def __init__( self ):
        self.channels = range(1,7)
        self.frequency = rospy.get_param( "~pwm_frequency", 50 )
        self.period = 1.0/self.frequency * 1000000 # period in microseconds
        self.pwm = Pwm( self.channels, self.frequency )
        self.pwm_topic = rospy.get_param( "~pwm_topic", "pwm" )
        self.pwm_sub = rospy.Subscriber( self.pwm_topic, falkor_msgs.msg.Pwm, self.pwm_cb )

    def pwm_cb(self, data):
        if len( data.pwm ) != len( self.channels ):
	    import pdb; pdb.set_trace()	
            raise Exception( "Length of pwm data doesn't match the number of channels" )

        for i in zip( self.channels, data.pwm ):
            # convert us to duty percent
            duty_percent = int( i[1] / self.period * 100 )
            self.pwm.setDutyPercentByChannel( i[0], duty_percent )
        

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

def main():
    rospy.init_node('pwm_beagle_driver')
    driver = PwmDriver()
    try:
        driver.run()
    except KeyboardInterrupt:
        print "Shutting down"

if __name__  == '__main__':
    main()
