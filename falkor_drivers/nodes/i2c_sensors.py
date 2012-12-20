import smbus,time
import rospy

def int16(b):
    thres = 1 << 15
    offset = 1 << 16
    if b >= thres:
        b -= offset
    
    return b

def int32(b):
    thres = 1 << 31
    offset = 1 << 32
    if b >= thres:
        b -= offset
    return b

class I2CSensors:
    def __init__(self, bus_number):
        self.gyro_addr = 0x68
        self.mag_addr = 0x1E
        self.accel_addr = 0x53
        self.baro_addr = 0x77
        self.bus = smbus.SMBus(bus_number)

        try:
            self.init_gyro()
        except Exception as e:
            rospy.logerr( "Gyro init failed: %s" % str(e) )
            self.gyro_working = False
        else:
            self.gyro_working = True
            self.gyro_avg = self.calib_gyro(60)

        try:
            self.init_accel()
        except Exception as e:
            rospy.logerr( "Accelerometer init failed: %s" % str(e) )
            self.accel_working = False
        else:
            self.accel_working = True
    
        try:
            self.init_mag()
        except Exception as e:
            rospy.logerr( "Magnetometer init failed: %s" % str(e) )
            self.mag_working = False
        else:
            self.mag_working = True

        try:
            self.init_baro()
        except Exception as e:
            rospy.logerr( "Barometer init failed: %s" % str(e) )
            self.baro_working = False
        else:
            self.baro_working = True

    def calib_gyro(self,n):
	# gyro runs at 125Hz
	rate = 125
	samples = 125*n
	gyro_avg = [0,0,0]
	for i in range(n*rate):
            gyro_vals = self.gyro_raw() 
	    for i in range(3):
                gyro_avg[i] += float(gyro_vals[i])/samples

            time.sleep(1.0/rate)
	print gyro_avg

	return gyro_avg

    def gyro_read(self):
        if self.gyro_working == False:
            return None

	raw = self.gyro_raw()
	calibrated = [0,0,0]
	for i in range(3):
		calibrated[i] = ( raw[i]-self.gyro_avg[i] ) / 14.375 / 180 * np.pi

	return calibrated

    def init_gyro(self):
	# Power up reset defaults
	self.bus.write_byte_data(self.gyro_addr,0x3E,0x80)
	time.sleep(0.005)
	# Select full-scale range of the gyro sensors
	# Set LP filter bandwidth to 256Hz
        # DLPF_CFG = 0, FS_SEL = 3
	self.bus.write_byte_data(self.gyro_addr,0x16,0x18)
	time.sleep(0.005)
	# Set sample rato to 125Hz
        # 8kHz / 64 = 125Hz
        # SMPLRT_DIV = 63
	self.bus.write_byte_data(self.gyro_addr,0x15,0x3F)
	time.sleep(0.005)
	# Set clock to PLL with z gyro reference
	self.bus.write_byte_data(self.gyro_addr,0x3E,0x00)
	time.sleep(0.005)

    def gyro_raw(self):
	while True:
	    try:
                buff = self.bus.read_i2c_block_data(self.gyro_addr,0x1d)
                break
            except IOError:
                rospy.logwarn( "Lost Connection while reading gyro" )

        x = -1 * int16((buff[2] << 8) | buff[3])
        y = -1 * int16((buff[0] << 8) | buff[1])
        z = -1 * int16((buff[4] << 8) | buff[5])
     
        return [x, y, z]

    def init_accel(self):
	self.bus.write_byte_data(self.accel_addr,0x2D,0x08) # Power register to Measurement Mode
	time.sleep(0.005)
	self.bus.write_byte_data(self.accel_addr,0x31,0x08) # Data format register set to full resolution
	time.sleep(0.005)
	self.bus.write_byte_data(self.accel_addr,0x2C,0x0A) # Set data rate 100 hz
	time.sleep(0.005)

    def accel_read(self):
        if self.accel_working == False:
            return None

        while True:
            try:
                buff = self.bus.read_i2c_block_data(self.accel_addr,0x32)
                break
	    except IOError:
                rospy.logwarn( "Lost Connection while reading accel" )

	x = int16((buff[3] << 8) | buff[2]) / 256.0 * 9.82
	y = int16((buff[1] << 8) | buff[0]) / 256.0 * 9.82
	z = int16((buff[5] << 8) | buff[4]) / 256.0 * 9.82

	return [x, y, z]

    def self_test_mag(self):
        test_limits = { 5: [ 243, 575 ],
                        6: [ 206, 487 ],
                        7: [ 143, 339 ] }

        # Configure self-test
        # 8-average, 15Hz, positive self-test
        self.bus.write_byte_data(self.mag_addr,0x00,0x71)
        time.sleep(0.005)

        # continuous measurement mode
        self.bus.write_byte_data(self.mag_addr,0x02,0x00)
        time.sleep(0.005)

        mag_test_success = False
        for gain in [5, 6, 7]:
            gain = 5

            # Set gain 
            self.bus.write_byte_data(self.mag_addr,0x01,gain << 5)
            time.sleep(0.005)

            # Read test-data
            # Read once and discard
            [x,y,z] = self.mag_read_raw()
            time.sleep(0.070)

            [x,y,z] = self.mag_read_raw()

            # Check limits
            if ( x > test_limits[gain][0] and x < test_limits[gain][1] and
                 y > test_limits[gain][0] and y < test_limits[gain][1] and
                 z > test_limits[gain][0] and z < test_limits[gain][1] ):
                mag_test_success = True
                break
            else:
                rospy.logwarn( "Magnetometer self test out of limits for gain: %d" % gain )

            time.sleep(0.070)

        if not mag_test_success:
            raise Exception( "Magnetometer self test failed" )
        else:
            rospy.loginfo( "magnetometer STP: (%d,%d,%d), gain = %d" %
                           ( x, y, z, gain ) )
            # if we have stored STP values from calibration
            #self.x_tempcomp = X_STP / x
            #self.y_tempcomp = Y_STP / y
            #self.z_tempcomp = Z_STP / z
            self.x_tempcomp = self.y_tempcomp = self.z_tempcomp = 1

    def init_mag(self):
        self.self_test_mag()

        # Continuous measurement mode
	self.bus.write_byte_data(self.mag_addr,0x02,0x00)
	time.sleep(0.005)

        # gain = 1090 (0.92 mG/LSB)
	self.bus.write_byte_data(self.mag_addr,0x02,0xA0)
	time.sleep(0.005)

        # 8 average, 15Hz, normal measurement
        self.bus.write_byte_data(self.mag_addr,0x00,0x70)
        time.sleep(0.005)

    def mag_read_raw(self):
        if self.mag_working == False:
            return None

	while True:
            try:
	        buff = self.bus.read_i2c_block_data(self.mag_addr,0x03)
		break
	    except IOError:
                rospy.logwarn( "Lost Connection while reading mag" )

	x = int16((buff[0] << 8) | buff[1])
	y = int16((buff[4] << 8) | buff[5])
	z = int16((buff[2] << 8) | buff[3])
        return [ x, y, z ]

    def mag_read(self):
        [x, y, z] = self.mag_read_raw()

        # Resolution for gain = 1 is 0.92 mG/LSB
        x *= 0.92e-3 * self.x_tempcomp
        y *= 0.92e-3 * self.y_tempcomp
        z *= 0.92e-3 * self.z_tempcomp
	return [x, y, z]

    def init_baro(self):
	buff = self.bus.read_i2c_block_data(self.baro_addr,0xAA)
	data = []
	data.append(int16((buff[0] << 8) | buff[1]))
	data.append(int16((buff[2] << 8) | buff[3]))
	data.append(int16((buff[4] << 8) | buff[5]))
	data.append((buff[6] << 8) | buff[7])
	data.append((buff[8] << 8) | buff[9])
	data.append((buff[10] << 8) | buff[11])
	data.append(int16((buff[12] << 8) | buff[13]))
	data.append(int16((buff[14] << 8) | buff[15]))
	data.append(int16((buff[16] << 8) | buff[17]))
	data.append(int16((buff[18] << 8) | buff[19]))
	data.append(int16((buff[20] << 8) | buff[21]))
        self.baro_data = data
        self.last_temp_read = None
        
        return data

    def load_temp(self):
	self.bus.write_byte_data(self.baro_addr,0xF4,0x2E)
	time.sleep(0.005)
	
    def load_pres(self,oss):
	timing = [0.005,0.008,0.014,0.026]
	cmd = 0x34 + (oss<<6)
	self.bus.write_byte_data(self.baro_addr,0xF4,cmd)
	time.sleep(timing[oss])
	
    def baro_read(self,oss=0):
        if self.baro_working == False:
            return None

	ac1 = self.baro_data[0]
	ac2 = self.baro_data[1]
	ac3 = self.baro_data[2]
	ac4 = self.baro_data[3]
	ac5 = self.baro_data[4]
	ac6 = self.baro_data[5]
	b1 = self.baro_data[6]
	b2 = self.baro_data[7]
	mb = self.baro_data[8]
	mc = self.baro_data[9]
	md = self.baro_data[10]
	
        # Only get the temperature once per second
        now = time.time()
        if self.last_temp_read == None or ( now - self.last_temp_read > 1 ):
            self.last_temp_read = now
            self.load_temp()
            while True:
                try:
                    buff = self.bus.read_i2c_block_data(self.baro_addr,0xF6)
                    break
                except IOError:
                    rospy.logwarn( "Lost connection while reading barometer" )
            
            ut = ((buff[0] << 8) | buff[1])

            # Calculating temperature
            x1 = ((ut - ac6) * ac5) >> 15
            x2 = (mc << 11) / (x1 + md)
            self.b5 = x1 + x2
            self.temperature = (self.b5 + 8) >> 4

	self.load_pres(oss)

	while True:
            try:
	        buff = self.bus.read_i2c_block_data(self.baro_addr,0xF6)
		break
	    except IOError:
                rospy.logwarn( "Lost connection while reading barometer" )

	up = ((buff[0] << 16) + (buff[1] << 8) + buff[2]) >> (8-oss)
	
	# Calculating pressure
	b6 = self.b5 - 4000
	x1 = (b2 * ((b6 * b6) >> 12)) >> 11
	x2 = (ac2 * b6)>> 11
	x3 = x1 + x2
	b3 = (((ac1 * 4 + x3) << oss) + 2) >> 2
	x1 = ac3 * b6 >> 13
	x2 = (b1 * ((b6 * b6) >>12)) >> 16
	x3 = ((x1 + x2) + 2) >> 2
	b4 = (ac4 * (x3 + 32768)) >> 15
	b7 = (up - b3) * (50000 >> oss)
	p = (b7 << 1) / b4
	
	x1 = (p >> 8) * (p >> 8)
	x1 = (x1 * 3038) >> 16
	x2 = int32(-7357 * p) >> 16
	pressure = p + (x1 + x2 + 3791) >> 4
	
	altitude = 44330*(1-(p/101325.0)**(1/5.255))
	return [self.temperature/10.0, pressure/1000.0, altitude]


