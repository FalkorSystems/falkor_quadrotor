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

        self.init_gyro()
        self.init_accel()
	self.init_mag()
	self.baro_data = self.init_baro()
	self.gyro_avg = self.calib_gyro(1000)

    def calib_gyro(self,n):
	gyro_avg = [0,0,0]
	for i in range(n):
            gyro_vals = self.gyro_read() 
	    for i in range(3):
                gyro_avg[i] += float(gyro_vals[i])/n

            # Gyro runs at 125Hz
            time.sleep(1/125)

	return gyro_avg

    def gyro_data(self):
	raw = self.gyro_read()
	calibrated = [0,0,0]
	for i in range(3):
		calibrated[i] = raw[i]-self.gyro_avg[i]
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

    def gyro_read(self):
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
        while True:
            try:
                buff = self.bus.read_i2c_block_data(self.accel_addr,0x32)
                break
	    except IOError:
                rospy.logwarn( "Lost Connection while reading accel" )

	x = int16((buff[3] << 8) | buff[2])
	y = int16((buff[1] << 8) | buff[0])
	z = int16((buff[5] << 8) | buff[4])
	return [x, y, z]

    def init_mag(self):
        # Continuous measurement mode
	self.bus.write_byte_data(self.mag_addr,0x02,0x00)
	time.sleep(0.005)

        # gain = 5
	self.bus.write_byte_data(self.mag_addr,0x01,0xA0)
	time.sleep(0.005)

        # 8 average, 15Hz, normal measurement
        self.bus.write_byte_data(self.mag_addr,0x00,0x70)
        time.sleep(0.005)


    def mag_read(self):
	while True:
            try:
	        buff = self.bus.read_i2c_block_data(self.mag_addr,0x03)
		break
	    except IOError:
                rospy.logwarn( "Lost Connection while reading mag" )

	x = int16((buff[0] << 8) | buff[1])
	y = int16((buff[4] << 8) | buff[5])
	z = int16((buff[2] << 8) | buff[3])
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

	return data

    def load_tem(self):
	self.bus.write_byte_data(self.baro_addr,0xF4,0x2E)
	time.sleep(0.005)
	
    def load_pres(self,oss):
	timing = [0.005,0.008,0.014,0.026]
	cmd = 0x34 + (oss<<6)
	self.bus.write_byte_data(self.baro_addr,0xF4,cmd)
	time.sleep(timing[oss])
	
    def baro_read(self):
	oss = 0
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
	
	self.load_tem()
	while True:
	    try:
	        buff = self.bus.read_i2c_block_data(self.baro_addr,0xF6)
		break
	    except IOError:
                rospy.logwarn( "Lost connection while reading barometer" )

	ut = ((buff[0] << 8) | buff[1])
	self.load_pres(oss)

	while True:
            try:
	        buff = self.bus.read_i2c_block_data(self.baro_addr,0xF6)
		break
	    except IOError:
                rospy.logwarn( "Lost connection while reading barometer" )

	up = ((buff[0] << 16) + (buff[1] << 8) + buff[2]) >> (8-oss)
	
	# Calculating temperature
	x1 = (ut - ac6) * ac5 / 2**15
	x2 = (mc * 2**11) / (x1 + md)
	b5 = x1 + x2
	temperature = (b5 + 8) / 2**4
	
	# Calculating pressure
	b6 = b5 - 4000
	x1 = (b2 * (b6 * b6 / 2**12)) / 2**11 
	x2 = ac2 * b6 / 2**11
	x3 = x1 + x2
	b3 = (((ac1 * 4 + x3) << oss) + 2) / 4
	x1 = ac3 * b6 / 2**13
	x2 = (b1 * (b6 * b6 / 2**12)) / 2**16
	x3 = ((x1 + x2) + 2) / 2**2
	b4 = (ac4 * (x3 + 32768)) / 2**15
	b7 = (up - b3) * (50000 >> oss)
	p = (b7 * 2) / b4
	
	x1 = int((p / 2.0**8)**2)
	x1 = (x1 * 3038) / 2**16
	x2 = int32(-7357 * p) / 2**16
	pressure = p + (x1 + x2 + 3791) / 2**4
	
	altitude = round(44330*(1-(p/101325.0)**(1/5.255)),1)
	return [temperature/10.0, pressure/1000.0, altitude]


