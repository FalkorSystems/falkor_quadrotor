#include <stdlib.h>
#include <falkor_drivers/i2c.h>
#include <falkor_drivers/sensor.h>
#include <iostream>
#include <unistd.h>
 
void print_vector( const std::vector<double> &vec )
{
  for( std::vector<double>::const_iterator iter = vec.begin(); iter < vec.end(); iter++ )
    {
      std::cout << *iter << " ";
    }
  std::cout << std::endl;
}

void init_and_print( I2CSensor *i2c_sensor )
{
  try {
    i2c_sensor->init();
    print_vector( i2c_sensor->read() );

    print_vector( i2c_sensor->read() );
  }
  catch( std::exception( e ) ) {
      std::cerr << e.what() << std::endl;
  }
}

int main(void)
{
  I2CDriver i2c(3);
  Barometer barometer( 0x77, &i2c);
  Accelerometer accelerometer( 0x53, &i2c );
  Magnetometer magnetometer( 0x1E, &i2c, 0.0, 0.0, 1.0, 1.0, 0.0, 466, 433, 453 );
  Gyrometer gyrometer( 0x68, &i2c );

  i2c.writeByteData( 0x1E, 0x01, 0x70 );
  
  std::cout << "baro" << std::endl;
  init_and_print( &barometer );

  std::cout << "accel" << std::endl;
  init_and_print( &accelerometer );

  std::cout << "magnet" << std::endl;
  init_and_print( &magnetometer );

  std::cout << "gyrometer" << std::endl;
  init_and_print( &gyrometer );
}
