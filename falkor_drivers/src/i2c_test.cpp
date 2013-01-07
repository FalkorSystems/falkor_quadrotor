#include <stdlib.h>
#include <falkor_drivers/i2c.h>
    
int main(void)
{
  I2CDriver i2c(3);
  std::vector<uint8_t> result = i2c.readI2CBlockData( 0x68, 0x1d, 32 );

  for( int i = 0; i < result.size(); i++ )
    printf( "%X ", result[i] );

  printf( "\n" );
}
