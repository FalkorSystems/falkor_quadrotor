#include <falkor_drivers/i2c.h>

I2CDriver::I2CDriver( uint8_t device )
{
  char path[MAX_PATH];

  snprintf( path, MAX_PATH, "/dev/i2c-%d", device );

  if( ( fd = open( path, O_RDWR, 0 ) ) == -1 )
    throw std::runtime_error( strerror( errno ) );

}

I2CDriver::~I2CDriver()
{
  close(fd);
}
  
void I2CDriver::setAddress( uint8_t address )
{
  int ret = 0;
  if( ( ret = ioctl( fd, I2C_SLAVE, address ) ) < 0 )
    throw std::runtime_error( strerror( errno ) );
}

void I2CDriver::writeQuick( uint8_t address )
{
  int32_t result;
  setAddress( address );
  if ((result = i2c_smbus_write_quick(fd, I2C_SMBUS_WRITE)))
    throw std::runtime_error( strerror( errno ) );

}

uint8_t I2CDriver::readByte( uint8_t address )
{
  int32_t result;
  setAddress( address );

  if((result = i2c_smbus_read_byte(fd)) == -1 )
    throw std::runtime_error( strerror( errno ) );

  uint8_t byte = result & 0xFF;
  return byte;
}

void I2CDriver::writeByte( uint8_t address, uint8_t value )
{
  setAddress( address );
  int32_t result;

  if((result = i2c_smbus_write_byte( fd, value )) == -1 )
    throw std::runtime_error( strerror( errno ) );

}

uint8_t I2CDriver::readByteData( uint8_t address, uint8_t cmd )
{
  int32_t result;

  setAddress( address );
  if((result = i2c_smbus_read_byte_data(fd, cmd)) == -1)
    throw std::runtime_error( strerror( errno ) );

  return result & 0xFF;
}

void I2CDriver::writeByteData( uint8_t address, uint8_t cmd, uint8_t val )
{
  int32_t result;

  setAddress( address );

  if ((result = i2c_smbus_write_byte_data( fd, cmd, val )) == -1 )
    throw std::runtime_error( strerror( errno ) );
}

uint16_t I2CDriver::readWordData( uint8_t address, uint8_t cmd )
{
  int32_t result;

  setAddress( address );
  if ((result = i2c_smbus_read_word_data(fd, cmd)) == -1 )
    throw std::runtime_error( strerror( errno ) );

  return result & 0xFFFF;
}

void I2CDriver::writeWordData( uint8_t address, uint8_t cmd, uint16_t val )
{
  int32_t result;
  setAddress( address );

  if((result = i2c_smbus_write_word_data(fd, cmd, val)) == -1 )
    throw std::runtime_error( strerror( errno ) );
}

std::vector<uint8_t> I2CDriver::dataToVector( uint8_t *buf, int len )
{
  std::vector<uint8_t> ret( len );
  for(int i = 0; i < len; i++)
    {
      ret[i] = buf[i];
    }
  return ret;
}

std::vector<uint8_t> I2CDriver::readI2CBlockData( uint8_t address, uint8_t cmd, int len = 32 )
{
  union i2c_smbus_data data;
  setAddress( address );

  data.block[0] = len;
  if( i2c_smbus_access(fd, I2C_SMBUS_READ, cmd,
		       len == 32 ? I2C_SMBUS_I2C_BLOCK_BROKEN:
		       I2C_SMBUS_I2C_BLOCK_DATA, &data ))
    throw std::runtime_error( strerror( errno ) );

  return dataToVector( &data.block[1], data.block[0] );
}

