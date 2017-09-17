#ifndef MPU_H
#define MPU_H

#include <wiringPi.h>
#include <wiringPiI2C.h>

#include <iostream>
#include <stdint.h>


#define SCALE_2000 2000.0/(256.0*128.0)

#define MPU_ADRESS 0x68
#define GYRO_RANGE_REG 0x1B
#define I2C_BYPASS 0x37
#define I2C_MST_EN 0x6A
#define PWR_MNGT1 0x6B

#define HMC_ADRESS 0x1E
#define MODE_REG 0x02
#define OUTPUT_REG 0x06

#define ACCEL_RANGE_REG 0x1C
#define ACCEL_RANGE_2G 2
#define ACCEL_RANGE_4G 4
#define ACCEL_RANGE_8G 8
#define ACCEL_RANGE_16G 16
#define ACCEL_DATA_X_H 0x3B
#define ACCEL_DATA_X_L 0x3C
#define ACCEL_DATA_Y_H 0x3D
#define ACCEL_DATA_Y_L 0x3E
#define ACCEL_DATA_Z_H 0x3F
#define ACCEL_DATA_Z_L 0x40

#define GYRO_SCALE_2000 2000.0/(256.0*128.0)*PI/180.0
#define GRAVITY 9.81
#define ACCEL_SCALE_2G 2.0*GRAVITY/32767.0
#define ACCEL_SCALE_4G 4.0*GRAVITY/32767.0
#define ACCEL_SCALE_8G 8.0*GRAVITY/32767.0
#define ACCEL_SCALE_16G 16.0*GRAVITY/32767.0

struct I2C_data{
  int16_t data1;
  int16_t data2;
  int16_t data3;
};

class Mag{
public:
  Mag();
  void GetDescriptor();
  void SetContinuosMode();
  I2C_data GetData();
  float constrainMagAngle(float mag_angle, float mag_offset, float x_out, float y_out);
  int mag_;
};

class MPU{
public:
  MPU();  
  void GetDescriptor();
  void Gyro_SetRange2000();
  void EnableI2CBypass();
  int16_t Gyro_GetDataZ();
  float constrainGyroAngle(float gyro_angle);

  void Accel_SetRange(int range);
  I2C_data Accel_GetData();
  int mpu_;
  double accel_scale_;
};


#endif // MPU_H
