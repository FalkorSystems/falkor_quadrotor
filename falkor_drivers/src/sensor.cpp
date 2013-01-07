#include <falkor_drivers/sensor.h>
#include <unistd.h>
#include <math.h>
#include <stdexcept>


std::vector<double> I2CSensor::read(void)
{
  if (!working)
    {
      // return a vector full of zeros? or an empty vector...???
      std::vector<double> result(3, 0.0);
      return result;
    }
  return readBase();
}    

void I2CSensor::init(void)
{
  try {
    initBase();
  }
  catch( std::exception(e) ) {
    working = false;
    return;
  }
  working = true;
}

I2CSensor::I2CSensor( uint8_t address_, I2CDriver *busPtr_ ) : address(address_),
							       busPtr( busPtr_ ), working( false )
{
}

I2CSensor::~I2CSensor()
{
}

Accelerometer::Accelerometer( uint8_t address_, I2CDriver *busPtr_ ) : I2CSensor( address_, busPtr_ )
{
}

void Accelerometer::initBase(void) {
  // Power register to measurement mode
  busPtr->writeByteData(address, 0x2D, 0x08);

  // Data format register set to full resolution
  busPtr->writeByteData(address, 0x31, 0x08);

  // Set data rate 100Hz
  busPtr->writeByteData(address, 0x2C, 0x0A);
}

std::vector<double> Accelerometer::readBase(void) {
  // Read 6 bytes from 0x32
  std::vector<uint8_t> data = busPtr->readI2CBlockData(address, 0x32, 6);

  std::vector<double> result(3);
  result[0] = int16_t( ( data[3] << 8 ) | data[2] ) / 256.0 * 9.82;
  result[1] = int16_t( ( data[1] << 8 ) | data[0] ) / 256.0 * 9.82;
  result[2] = int16_t( ( data[5] << 8 ) | data[4] ) / 256.0 * 9.82;

  return result;
}

Magnetometer::Magnetometer( uint8_t address_, I2CDriver *busPtr_, double center_x_, double center_y_,
			    double axes_x_, double axes_y_, double angle_, double X_STP_, double Y_STP_,
			    double Z_STP_ ) : I2CSensor( address_, busPtr_ )
{
  center[0] = center_x_;
  center[1] = center_y_;
  axes[0] = axes_x_;
  axes[1] = axes_y_;
  angle = angle_;
  cos_angle = cos( angle );
  sin_angle = sin( angle );
  STP[0] = X_STP_;
  STP[1] = Y_STP_;
  STP[2] = Z_STP_;
}

std::vector<int16_t> Magnetometer::readRaw(void)
{
  std::vector<uint8_t> data = busPtr->readI2CBlockData(address, 0x03, 6);
  std::vector<int16_t> result(3);

  result[0] = int16_t( ( data[0]<<8 ) | data[1] );
  result[1] = int16_t( ( data[4]<<8 ) | data[5] );
  result[2] = int16_t( ( data[2]<<8 ) | data[3] );
  return result;
}

std::vector<double> Magnetometer::tempCompensate( const std::vector<int16_t> &rawResult ) {
  std::vector<double> result(3);
  for( int i=0; i < 3; i++ )
    {
      result[i] = rawResult[i] * 0.92e-3 * tempComp[i];
    }
  return result;
}

void Magnetometer::selfTest(void)
{
  uint16_t test_limits[3][2] = { { 245, 575 },
				 { 206, 487 },
				 { 143, 339 } };
  uint8_t test_gains[] = { 5, 6, 7 };
    
  // Configure self_test
  busPtr->writeByteData( address, 0x00, 0x71 );
  busPtr->writeByteData( address, 0x02, 0x00 );

  int magTestSuccess = true;
  std::vector<int16_t> rawData;

  for( int i = 0; i < 3; i++ )
    {
      uint8_t gain = test_gains[i];
      busPtr->writeByteData( address, 0x01, gain << 5 );

      // Read and discard
      readRaw();
      
      rawData = readRaw();

      
      magTestSuccess = true;
      for( int j = 0; j < 3; j++ )
	{
	  if( ( rawData[j] < test_limits[i][0] ) || ( rawData[j] > test_limits[i][1] ) )
	    magTestSuccess = false;
	}

      if( magTestSuccess )
	break;
    }
  if( !magTestSuccess )
    throw std::runtime_error( "Mag self test failure" );

  for( int i = 0; i < 3; i++ )
    {
      tempComp[i] = double( STP[i] ) / rawData[i];
    }
}

std::vector<double> Magnetometer::correct( const std::vector<double> &uncorrected )
{
  std::vector<double> result(3);
  result[0] = uncorrected[0] - center[0];
  result[1] = uncorrected[1] - center[1];
  double x = result[0] * cos_angle + result[1] * sin_angle;
  double y = - result[0] * sin_angle + result[1] * cos_angle;
  result[0] = x / axes[0];
  result[1] = y / axes[1];
  result[2] = uncorrected[2];
  return result;
}

std::vector<double> Magnetometer::readBase(void)
{
  std::vector<int16_t> rawResult = readRaw();
  std::vector<double> resultTemp = tempCompensate( rawResult );
  //      std::vector<double> resultCalibrated = correct( resultTemp );

  return resultTemp;
}
      
void Magnetometer::initBase(void)
{
  selfTest();

  // Continuous measurement mode
  busPtr->writeByteData( address, 0x02, 0x00 );

  // Gain = 1090 (0.92 mG/LSB)
  busPtr->writeByteData( address, 0x01, 0x20 );

  // 8 average, 15Hz, normal measurement
  busPtr->writeByteData( address, 0x00, 0x70 );

  // Take a reading and ignore it
  readRaw();
}

Gyrometer::Gyrometer( uint8_t address_, I2CDriver *busPtr_ ) : I2CSensor( address_, busPtr_ )
{
}

void Gyrometer::initBase(void) {
  // Power up reset defaults
  busPtr->writeByteData(address, 0x3E, 0x80);

  // Select full-scale ralte of Gyro sensors
  // set LP filter bandwidth to 256Hz
  // DLPF_CFG = 0, FS_SEL = 3
  busPtr->writeByteData(address, 0x16, 0x18);

  // Set sample reatio to 125Hz
  // 8 kHz/64 = 125Hz
  // SMPLRT_DIV = 63
  busPtr->writeByteData(address, 0x15, 0x3F);

  // Set clock to PLL with z gyro reference
  busPtr->writeByteData(address, 0x3E, 0x00);

  calibrate(60);
}

std::vector<uint16_t> Gyrometer::readRaw(void) {
  std::vector<uint8_t> data = busPtr->readI2CBlockData(address, 0x1d, 6);
  std::vector<uint16_t> raw(3);

  raw[0] = -1 * int16_t(( data[2] << 8 ) | data[3] );
  raw[1] = -1 * int16_t(( data[0] << 8 ) | data[1] );
  raw[2] = -1 * int16_t(( data[4] << 8 ) | data[5] );

  return raw;
}

void Gyrometer::calibrate(int n) {
  int rate = 125;
  int samples = rate * n;
  useconds_t sleep_time = 1000000 / rate;

  gyroCalibration[0] = gyroCalibration[1] = gyroCalibration[2] = 0;
  for(int i = 0; i < samples; i++)
    {
      std::vector<uint16_t> gyro_vals = readRaw();
      for(int j = 0; j < 3; j++)
	  gyroCalibration[j] += double(gyro_vals[j])/samples;

      if( usleep( sleep_time ) == -1 )
	throw std::runtime_error( strerror( errno ) );
    }
}

std::vector<double> Gyrometer::readBase(void) {
  std::vector<uint16_t> raw = readRaw();
  std::vector<double> result(3);
  for(int i = 0; i < 3; i++)
    result[i] = ( raw[i] - gyroCalibration[i] ) / 14.375 / 180 * M_PI;

  return result;
}

Barometer::Barometer( uint8_t address_, I2CDriver *busPtr_ ) : I2CSensor( address_, busPtr_ )
{
}

void Barometer::initBase(void) {
  loadCalibration();
  lastTemperatureTime.tv_sec = 0;
  lastTemperatureTime.tv_usec = 0;
}

void Barometer::loadCalibration(void) {
  std::vector<uint8_t> data = busPtr->readI2CBlockData( address, 0xAA, 22 );
  ac1 = int16_t(( data[0] << 8 ) | data[1] );
  ac2 = int16_t(( data[2] << 8 ) | data[3] );
  ac3 = int16_t(( data[4] << 8 ) | data[5] );
  ac4 = uint16_t(( data[6] << 8 ) | data[7] );
  ac5 = uint16_t(( data[8] << 8 ) | data[9] );
  ac6 = uint16_t(( data[10] << 8 ) | data[11] );
  b1 = int16_t(( data[12] << 8 ) | data[13] );
  b2 = int16_t(( data[14] << 8 ) | data[15] );
  mb = int16_t(( data[16] << 8 ) | data[17] );
  mc = int16_t(( data[18] << 8 ) | data[19] );
  md = int16_t(( data[20] << 8 ) | data[21] );
}

void Barometer::loadTemperature(void) {
  busPtr->writeByteData(address, 0xF4, 0x2E);
}

void Barometer::loadPressure(uint8_t oss) {
  if( oss > 3 )
    throw std::runtime_error( "oss exceeds 3" );

  useconds_t timing[] = { 5000, 8000, 14000, 26000 };
  uint8_t cmd = 0x34 + (oss<<6);

  busPtr->writeByteData(address, 0xF4, cmd);
  if( usleep( timing[oss] ) == -1 )
    throw std::runtime_error( strerror( errno ) );
}

std::vector<double> Barometer::readBase(void) {
  struct timeval now;
  uint8_t oss = 3;

  if( gettimeofday( &now, NULL ) == -1 )
    throw std::runtime_error( strerror( errno ) );

  // If a second has elapsed, get a new temperature
  if( ( now.tv_sec - lastTemperatureTime.tv_sec > 1 ) ||
      ( now.tv_sec - lastTemperatureTime.tv_sec > 0 &&
	now.tv_usec > lastTemperatureTime.tv_usec ) )
    {
      loadTemperature();
      std::vector<uint8_t> data = busPtr->readI2CBlockData(address, 0xF6, 2);
      uint16_t ut = ((data[0] << 8) | data[1] );

      double x1 = ( ut - ac6 ) * ac5 / 32768.0;
      double x2 = ( mc * 2048.0 ) / ( x1 + md );
      double b5 = x1 + x2;

      double b6 = b5 - 4000;
      x1 = (b2 * ( b6 * b6 / 4096.0 ) ) / 2048.0;
      x2 = ac2 * b6 / 2048.0;
      double x3 = x1 + x2;
      b3 = (((ac1 * 4 + x3) * pow(2,oss) ) + 2) / 4.0;
      x1 = ac3 * b6 / 8192.0;
      x2 = (b1 * ( b6 * b6 / 4096.0 )) / 65536.0;
      x3 = ((x1 + x2) + 2) / 4.0;
      b4 = (ac4 * (x3 * 32768.0))/32768.0;

      temperature = ( b5 + 8 ) / 16.0;
      lastTemperatureTime = now;
    }

  loadPressure(oss);

  std::vector<uint8_t> data = busPtr->readI2CBlockData(address, 0xF6, 3 );
  uint32_t up = ((data[0] << 16) | (data[1]<<8) | data[2] ) >> (8-oss);

  double b7 = (up - b3) * (50000>>oss);
  double p = (b7 * 2) / b4;
  double x1 = floor((p/256.0) * (p/256.0));
  x1 = (x1 * 3038) / 65536.0;
  double x2 = int32_t(-7537*p) / 65536.0;
  double pressure = p + (x1 + x2 + 3791) / 16.0;
  double altitude = 44330 * ( 1-pow((pressure/101325.0),1/5.255));

  std::vector<double> result(3);
  result[0] = temperature/10.0;
  result[1] = pressure/1000.0;
  result[2] = altitude;

  return result;
}

      
  