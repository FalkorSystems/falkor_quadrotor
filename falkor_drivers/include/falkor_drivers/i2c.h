// This is modeled after python-smbus code
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>
#include <linux/i2c-dev.h>
#include <exception>
#include <stdexcept>
#include <vector>
#include <errno.h>

#define MAX_PATH 16

class I2CDriver {
private:
    int fd;
    std::vector<uint8_t> dataToVector( uint8_t *buf, int len );

public:
    I2CDriver( uint8_t device );
    ~I2CDriver();

    void setAddress( uint8_t address );
    void writeQuick( uint8_t address );
    uint8_t readByte( uint8_t address );
    void writeByte( uint8_t address, uint8_t value );
    uint8_t readByteData( uint8_t address, uint8_t cmd );
    void writeByteData( uint8_t address, uint8_t cmd, uint8_t val );
    uint16_t readWordData( uint8_t address, uint8_t cmd );
    void writeWordData( uint8_t address, uint8_t cmd, uint16_t val );
    std::vector<uint8_t> readI2CBlockData( uint8_t address, uint8_t cmd, int len );
};
