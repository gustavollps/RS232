#include "../../include/heading/mpu.h"

/**
 * @brief Mag::Mag
 */
Mag::Mag() {}

/**
 * @brief Mag::GetDescriptor
 */
void Mag::GetDescriptor() { mag_ = wiringPiI2CSetup(HMC_ADRESS); }

/**
 * @brief Mag::SetContinuosMode
 */
void Mag::SetContinuosMode()
{
  wiringPiI2CWriteReg8(mag_, MODE_REG, 0); // Continuos reading mode
}

/**
 * @brief Mag::constrainMagAngle
 * @param mag_angle
 * @param mag_offset
 * @param x_out
 * @param y_out
 * @return
 */
float Mag::constrainMagAngle(float mag_angle, float mag_offset, float x_out,
                             float y_out)
{
  float angle = mag_angle;
  if (x_out < 0)
  {
    if (y_out > 0)
    {
      angle += 180;
    }
    else
    {
      angle -= 180;
    }
  }
  angle -= mag_offset;
  if (angle > 180)
  {
    angle -= 360;
  }
  else if (angle < (-180))
  {
    angle += 360;
  }
  return angle;
}

/**
 * @brief Mag::GetData
 * @return
 */
I2C_data Mag::GetData()
{
  I2C_data data;
  data.data1 = ((wiringPiI2CReadReg8(mag_, 0x03) << 8) |
                wiringPiI2CReadReg8(mag_, 0x04));
  data.data2 = ((wiringPiI2CReadReg8(mag_, 0x05) << 8) |
                wiringPiI2CReadReg8(mag_, 0x06));
  data.data3 = ((wiringPiI2CReadReg8(mag_, 0x07) << 8) |
                wiringPiI2CReadReg8(mag_, 0x08));

  // std::cout << data.data1 << " - " << data.data2 << " - " << data.data3 <<
  // std::endl;

  return data;
}

//---------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------------------------

/**
 * @brief MPU::MPU
 */
MPU::MPU() {}

/**
 * @brief MPU::GetDescriptor
 */
void MPU::GetDescriptor()
{
  mpu_ = wiringPiI2CSetup(MPU_ADRESS);
  wiringPiI2CWriteReg8(mpu_, PWR_MNGT1, 0); // Clear PWR_MNGT1 register, turn on
}

/**
 * @brief MPU::Gyro_SetRange2000
 */
void MPU::Gyro_SetRange2000()
{
  wiringPiI2CWriteReg8(mpu_, GYRO_RANGE_REG,
                       wiringPiI2CReadReg8(mpu_, GYRO_RANGE_REG) |
                           (0b00011000));
}

/**
 * @brief MPU::EnableI2CBypass
 */
void MPU::EnableI2CBypass()
{
  wiringPiI2CWriteReg8(mpu_, I2C_BYPASS, wiringPiI2CReadReg8(mpu_, I2C_BYPASS) |
                                             (0b00000010)); // SET I2C_BYPASS_EN
  wiringPiI2CWriteReg8(mpu_, I2C_MST_EN, wiringPiI2CReadReg8(mpu_, I2C_MST_EN) &
                                             ~(0b00010000)); // CLEAR I2C_MST_EN
}

/**
 * @brief MPU::Gyro_GetDataZ
 * @return
 */
int16_t MPU::Gyro_GetDataZ()
{
  int raw_data = ((wiringPiI2CReadReg8(mpu_, 0x47) << 8) |
                  (wiringPiI2CReadReg8(mpu_, 0x48)));
  return raw_data;
}

/**
 * @brief MPU::constrainGyroAngle
 * @param gyro_angle
 * @return
 */
float MPU::constrainGyroAngle(float gyro_angle)
{
  if (gyro_angle > 180)
  {
    gyro_angle -= 360;
  }
  else if (gyro_angle < (-180))
  {
    gyro_angle += 360;
  }
  return gyro_angle;
}

/**
 * @brief MPU::Accel_SetRange
 * @param range
 */
void MPU::Accel_SetRange(int range)
{

  if (range == ACCEL_RANGE_2G)
  { // bitclear 0b00011000
    wiringPiI2CWriteReg8(mpu_, ACCEL_RANGE_REG,
                         wiringPiI2CReadReg8(mpu_, ACCEL_RANGE_REG) &
                             ~(0b00011000));
    accel_scale_ = ACCEL_SCALE_2G;
  }
  else if (range == ACCEL_RANGE_4G)
  { // bitset 000010000 bitclear 00010000
    wiringPiI2CWriteReg8(mpu_, ACCEL_RANGE_REG,
                         wiringPiI2CReadReg8(mpu_, ACCEL_RANGE_REG) |
                             (0b00001000));
    wiringPiI2CWriteReg8(mpu_, ACCEL_RANGE_REG,
                         wiringPiI2CReadReg8(mpu_, ACCEL_RANGE_REG) &
                             ~(0b00010000));
    accel_scale_ = ACCEL_SCALE_4G;
  }
  else if (range == ACCEL_RANGE_8G)
  { // bitset 000100000 bitclear 000010000
    wiringPiI2CWriteReg8(mpu_, ACCEL_RANGE_REG,
                         wiringPiI2CReadReg8(mpu_, ACCEL_RANGE_REG) |
                             (0b00010000));
    wiringPiI2CWriteReg8(mpu_, ACCEL_RANGE_REG,
                         wiringPiI2CReadReg8(mpu_, ACCEL_RANGE_REG) &
                             ~(0b00001000));
    accel_scale_ = ACCEL_SCALE_8G;
  }
  else if (range = ACCEL_RANGE_16G)
  { // bitset 000110000
    wiringPiI2CWriteReg8(mpu_, ACCEL_RANGE_REG,
                         wiringPiI2CReadReg8(mpu_, ACCEL_RANGE_REG) |
                             (0b00011000));
    accel_scale_ = ACCEL_SCALE_16G;
  }
  else
  { // Defaul 2G
    wiringPiI2CWriteReg8(mpu_, ACCEL_RANGE_REG,
                         wiringPiI2CReadReg8(mpu_, ACCEL_RANGE_REG) &
                             ~(0b00011000));
    accel_scale_ = ACCEL_SCALE_2G;
  }  
}

/**
 * @brief MPU::Accel_GetData
 * @return
 */
I2C_data MPU::Accel_GetData()
{
  I2C_data data;
  data.data1 = ((wiringPiI2CReadReg8(mpu_, ACCEL_DATA_X_H) << 8) |
                wiringPiI2CReadReg8(mpu_, ACCEL_DATA_X_L));
  data.data2 = ((wiringPiI2CReadReg8(mpu_, ACCEL_DATA_Y_H) << 8) |
                wiringPiI2CReadReg8(mpu_, ACCEL_DATA_Y_L));
  data.data3 = ((wiringPiI2CReadReg8(mpu_, ACCEL_DATA_Z_H) << 8) |
                wiringPiI2CReadReg8(mpu_, ACCEL_DATA_Z_L));

  return data;
}
