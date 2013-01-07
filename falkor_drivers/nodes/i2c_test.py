#!/usr/bin/env python
from i2c_sensors import *

i2c = I2CSensors(3);
#print "baro ", i2c.baro_read(3)
#print "baro ", i2c.baro_read(3)

#print "accel ", i2c.accel_read()
#print "magnet ", i2c.mag_read()
print "gyro ", i2c.gyro_read()
print "gyro ", i2c.gyro_read()
