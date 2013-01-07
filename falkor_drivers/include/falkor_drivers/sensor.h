#ifndef FALKOR_DRIVERS_SENSOR_H
#define FALKOR_DRIVERS_SENSOR_H

#include <falkor_drivers/i2c.h>
#include <sys/time.h>

class I2CSensor
{
 protected:
  uint8_t address;
  I2CDriver *busPtr;
  int working;

 public:
  virtual void initBase(void) = 0;
  virtual std::vector<double> readBase(void) = 0;

  std::vector<double> read(void);
  void init(void);
  I2CSensor( uint8_t address_, I2CDriver *busPtr_ );
  ~I2CSensor();
};

class Accelerometer : public I2CSensor {
 public:
  Accelerometer( uint8_t address_, I2CDriver *busPtr_ );
  void initBase(void);
  std::vector<double> readBase(void);
};

class Magnetometer : public I2CSensor {
 private:
  double tempComp[3];
  double center[2];
  double axes[2];
  double angle, cos_angle, sin_angle;
  uint16_t STP[3];

  std::vector<int16_t> readRaw(void);
  std::vector<double> tempCompensate( const std::vector<int16_t> &rawResult );
  void selfTest(void);
  std::vector<double> correct( const std::vector<double> &uncorrected );

 public:
  Magnetometer( uint8_t address_, I2CDriver *busPtr_, double center_x_, double center_y_,
		double axes_x_, double axes_y_, double angle_, double X_STP_, double Y_STP_,
		double Z_STP_ );
  std::vector<double> readBase(void);
  void initBase(void);
};

class Gyrometer : public I2CSensor {
 private:
  double gyroCalibration[3];
  void calibrate(int n);
  std::vector<uint16_t> readRaw(void);

 public:
  Gyrometer( uint8_t address_, I2CDriver *busPtr_ );
  std::vector<double> readBase(void);
  void initBase(void);
};

class Barometer : public I2CSensor {
 private:
  // Calibration data;
  int16_t ac1, ac2, ac3;
  uint16_t ac4, ac5, ac6;
  int16_t b1, b2, mb, mc, md;

  // Temperature data
  double b3, b4;

  void loadCalibration(void);
  void loadTemperature(void);
  void loadPressure(uint8_t oss);

  struct timeval lastTemperatureTime;
  double temperature;

 public:
  Barometer( uint8_t address_, I2CDriver *busPtr_ );
  std::vector<double> readBase(void);
  void initBase(void);
};


#endif /* FALKOR_DRIVERS_SENSOR_H */
