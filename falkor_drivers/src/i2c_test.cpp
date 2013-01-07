#include <falkor_drivers/i2c.h>
    
int main(void)
{
  I2CDriver i2c(3);

  i2c.writeByte( 0x70, 0x51 );
}
