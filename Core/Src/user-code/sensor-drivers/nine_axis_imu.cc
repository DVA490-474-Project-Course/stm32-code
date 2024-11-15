/* nine_axis_imu.c
 *==============================================================================
 * Author: Emil Åberg
 * Creation date: 2024-11-11
 * Last modified: 2024-11-15 by Emil Åberg
 * Description: Driver for the nine axis BNO055 IMU.
 * License: See LICENSE file for license details.
 *==============================================================================
 */

/* Related .h files */
#include "nine_axis_imu.h"

/* C++ standard library headers */
#include <string.h>
#include <stdio.h>

/* Project .h files */
#include "../stm32h7xx_hal.h"
#include "read_write_functions.h"

namespace stm32_code
{
namespace sensor_drivers
{

NineAxisIMU::NineAxisIMU()
{

}

NineAxisIMU::~NineAxisIMU()
{

}

Status NineAxisIMU::Init(I2C_HandleTypeDef* hi2c)
{
  Status status;

  this->hi2c = hi2c;
  status = WriteByte(hi2c, i2c_address, opr_mode, ndof);
  HAL_Delay(50);
  return status;
}

Scalar<uint8_t> NineAxisIMU::GetTemperature()
{
  Scalar<uint8_t> temperature;
  uint8_t value;

  temperature.status = ReadByte(hi2c, i2c_address, temp, &value);
  temperature.value = ConvertToSigned(value);
  return temperature;
}

Vector3d<float> NineAxisIMU::GetMagneticField()
{
  return Get3dVector(mag_data_x_lsb, 16.0F);
}

Vector3d<float> NineAxisIMU::GetAcceleration()
{
  return Get3dVector(acc_data_x_lsb, 100.0F);
}

Vector3d<float> NineAxisIMU::GetRotationalAcceleration()
{
  return Get3dVector(gyr_data_x_lsb, 16.0F);
}

Vector3d<float> NineAxisIMU::GetLinearAcceleration()
{
  return Get3dVector(lia_data_x_lsb, 100.0F);
}

Rotation<float> NineAxisIMU::GetEulerOrientation()
{
  Rotation<float> rotation;

  uint8_t buffer[6];
  rotation.status = ReadBytes(hi2c, i2c_address, eul_heading_lsb, buffer, 6);
  rotation.heading = ((float)ConvertToSigned(Join(buffer[0], buffer[1]))) / 16.0F;
  rotation.roll = ((float)ConvertToSigned(Join(buffer[2], buffer[3]))) / 16.0F;
  rotation.pitch = ((float)ConvertToSigned(Join(buffer[4], buffer[5]))) / 16.0F;

  return rotation;
}

Quarternion<float> NineAxisIMU::GetQuarternionOrientation()
{
  Quarternion<float> quarternion;

  uint8_t buffer[8];
  quarternion.status = ReadBytes(hi2c, i2c_address, qua_data_w_lsb, buffer, 8);
  quarternion.w = ((float)ConvertToSigned(Join(buffer[0], buffer[1]))) / (float)(2^24);
  quarternion.x = ((float)ConvertToSigned(Join(buffer[2], buffer[3]))) / (float)(2^24);
  quarternion.y = ((float)ConvertToSigned(Join(buffer[4], buffer[5]))) / (float)(2^24);
  quarternion.z = ((float)ConvertToSigned(Join(buffer[6], buffer[7]))) / (float)(2^24);

  return quarternion;
}

Vector3d<float> NineAxisIMU::Get3dVector(const uint8_t register_address, float factor)
{
  Vector3d<float> vector;

  uint8_t buffer[6];
  vector.status = ReadBytes(hi2c, i2c_address, register_address, buffer, 6);
  vector.x = ((float)ConvertToSigned(Join(buffer[0], buffer[1]))) / factor;
  vector.y = ((float)ConvertToSigned(Join(buffer[2], buffer[3]))) / factor;
  vector.z = ((float)ConvertToSigned(Join(buffer[4], buffer[5]))) / factor;
  return vector;
}

Vector3d<float> NineAxisIMU::GetGravity()
{
  return Get3dVector(grv_data_x_lsb, 100.0F);
}

uint16_t NineAxisIMU::Join(uint8_t lsb, uint8_t msb)
{
  return lsb + (msb << 8);
}


int16_t NineAxisIMU::ConvertToSigned(uint16_t value)
{
  int16_t sign_mask = 0x8000;

  if (value & sign_mask)
  {
    return value;
  }
  else
  {
    return -(~value + 1);
  }
}

int8_t NineAxisIMU::ConvertToSigned(uint8_t value)
{
  int8_t sign_mask = 0x80;

  if (value & sign_mask)
  {
    return value;
  }
  else
  {
	return -(~value + 1);
  }
}

} /* namespace sensor_drivers */
} /* namespace stm32_code */




