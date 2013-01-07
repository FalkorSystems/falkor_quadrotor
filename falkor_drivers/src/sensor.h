#include <falkor_drivers/i2c.h>

class I2CSensor
{
 protected:
  I2CDriver *busPtr;
  uint8_t address;
  int working;

 public:
  virtual void initBase(void);
  virtual std::vector<double> readBase(void);

  std::vector<double> read(void) {
    if (!working)
      {
	// return an empty vector
	std::vector<double> result;
	return result;
      }
    return readBase();
  }    

  void init(void)
  {
    try {
      initBase();
    }
    catch( std::exception(e) ) {
      working = false;
    }
    else {
      working = true;
    }
  }

 I2CSensor( uint8_t address_, I2CDriver *busPtr_ ) : address(address_),
    busPtr( busPtr_ ), working( false )
    {
    }

  ~I2CSensor() {};
};

class Accelerometer : public I2CSensor {
 public:
 Accelerometer( uint8_t address_, I2CDriver *busPtr_ ) : I2CSensor( address_, busPtr_ )
    {
    }

  void initBase(void) {
    // Power register to measurement mode
    busPtr->writeByteData(address, 0x2D, 0x08);

    // Data format register set to full resolution
    busPtr->writeByteData(address, 0x31, 0x08);

    // Set data rate 100Hz
    busPtr->writeByteData(address, 0x2C, 0x0A);
  }

  std::vector<double> readBase(void) {
    // Read 6 bytes from 0x32
    std::vector<uint8_t> data = busPtr->readI2CBlockData(address, 0x32, 6);

    std::vector<double> result(3);
    result[0] = int16_t( ( data[3] << 8 ) | data[2] ) / 256.0 * 9.82;
    result[1] = int16_t( ( data[1] << 8 ) | data[0] ) / 256.0 * 9.82;
    result[2] = int16_t( ( data[5] << 8 ) | data[4] ) / 256.0 * 9.82;

    return result;
  }
}

class Magnetometer : public I2CSensor {
 private:
  double tempComp[3];
  double center[2];
  double axes[2];
  double angle, cos, sin;
  uint16_t STP[3];

  Magnetometer( uint8_t address_, I2CDriver *busPtr_, double center_x_, double center_y_,
		double axes_x_, double axes_y_, double angle_, double X_STP_, double Y_STP_,
		double Z_STP_ ) : I2CSensor( address_, busPtr_ )
    {
      center[0] = center_x_;
      center[1] = center_y_;
      axes[0] = axes_x_;
      axes[1] = axes_y_;
      angle = angle_;
      cos = cos( angle );
      sin = sin( angle );
      STP[0] = X_STP_;
      STP[1] = Y_STP_;
      STP[2] = Z_STP_;
    }

  std::vector<int16_t> readRaw(void)
    {
      std::vector<uint8_t> data = busPtr->readI2CBlockData(address, 0x03, 6);
      std::vector<int16_t> result(3);

      result[0] = int16_t( ( data[0]<<8 ) | data[1] );
      result[1] = int16_t( ( data[4]<<8 ) | data[5] );
      result[2] = int16_t( ( data[2]<<8 ) | data[3] );
      return result;
    }

  std::vector<double> tempCompensate( const std::vector<int16_t> &rawResult ) {
    std::vector<double> result(3);
    for( int i=0; i < 3; i++ )
      {
	result[i] = rawResult[i] * 0.92e-3 * tempComp[i];
      }
  }

  void selfTest(void)
  {
    uint16_t test_limits[][] = [ [ 245, 575 ],
				[ 206, 487 ],
				[ 143, 339 ] ];
    uint8_t test_gains[] = [ 5, 6, 7 ];
    uint16_t STP[] = [ 466, 433, 453 ];
    
    // Configure self_test
    busPtr->writeByteData( address, 0x00, 0x71 );
    busPtr->writeByteData( address, 0x02, 0x00 );

    for( int i = 0; i < 3; i++ )
      {
	uint8_t gain = test_gains[i];
	busPtr->writeByteData( address, 0x01, gain << 5 );

	// Read and discard
	readRaw();

	std::vector<int16_t> rawData = readRaw();
	int magTestSuccess = true;
	for( j = 0; j < 3; j++ )
	  {
	    if( rawData[j] < test_limits[i][0] || rawData[j] > test_limits[i][1] )
	      magTestSuccess = false;
	  }

	if( magTestSuccess )
	  break;
      }
    if( !magTestSuccess )
      throw std::runtime_exception( "Mag self test failure" );

    tempComp = std::vector<double>(3);
    for( int i = 0; i < 3; i++ )
      {
	tempComp[i] = double( STP[i] ) / rawData[i];
      }
  }

  std::vector<double> correct( const std::vector<double> &uncorrected )
    {
      std::vector<double> result(3);
      result[0] = uncorrected[0] - center[0];
      result[1] = uncorrected[1] - center[1];
      double x = result[0] * cos + result[1] * sin;
      double y = - result[0] * sin + result[1] * cos;
      result[0] = x / axes[0];
      result[1] = y / axes[1];
      result[2] = uncorrected[2];
      return result;
    }

 public:
  std::vector<double> readBase(void)
    {
      std::vector<uint16_t> rawResult = readRaw();
      std::vector<double> resultTemp = tempCompensate( rawResult );
      //      std::vector<double> resultCalibrated = correct( resultTemp );

      return resultTemp;
    }
      
  void initBase(void)
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
