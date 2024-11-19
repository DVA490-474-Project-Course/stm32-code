/* nine_axis_imu.c
 *==============================================================================
 * Author: Emil Åberg
 * Creation date: 2024-11-11
 * Last modified: 2024-11-19 by Emil Åberg
 * Description: Driver for the nine axis BNO055 IMU.
 * License: See LICENSE file for license details.
 *==============================================================================
 */

/* Related .h file */
#include "../sensor-interface/nine_axis_imu.h"

/* Project .h files */
#include "../common_types.h"
#include "../sensor-interface/read_write_functions.h"
#include "../stm32h7xx_hal.h"

namespace stm32_code
{
namespace sensor_interface
{

/* Default constructor */
NineAxisIMU::NineAxisIMU() {}

/* Initialise the nine axis IMU */
Status NineAxisIMU::Init(I2C_HandleTypeDef* hi2c)
{
  Status status;

  this->hi2c = hi2c;
  status = WriteByte(hi2c, i2c_address, opr_mode, ndof);
  HAL_Delay(50);
  return status;
}

/* Returns the measured temperature in degrees celsius */
Scalar<uint8_t> NineAxisIMU::GetTemperature()
{
  Scalar<uint8_t> temperature;
  uint8_t value;

  temperature.status = ReadByte(hi2c, i2c_address, temp_data, &value);
  temperature.value = ConvertToSigned(value);
  return temperature;
}

/* Returns the measured magnetic field in micro tesla */
Vector3d<float> NineAxisIMU::GetMagneticField()
{
  return Get3dVector(mag_data, mag_lsb_per_unit);
}

/* Returns the measured acceleration in m/s², acceleration due to gravity will
 * be included */
Vector3d<float> NineAxisIMU::GetAcceleration()
{
  return Get3dVector(acc_data, acc_lsb_per_unit);
}

/* Returns the measured rotational speed in degrees/s */
Vector3d<float> NineAxisIMU::GetRotationalSpeed()
{
  return Get3dVector(gyr_data, gyr_lsb_per_unit);
}

/* Returns the measured acceleration in m/s², acceleration due to gravity is
 * compensated for and will not be included */
Vector3d<float> NineAxisIMU::GetLinearAcceleration()
{
  return Get3dVector(lia_data, lia_lsb_per_unit);
}

/* Returns the measured gravity vector in m/s² */
Vector3d<float> NineAxisIMU::GetGravity()
{
  return Get3dVector(grv_data, grv_lsb_per_unit);
}

/* Returns the measured orientation in degrees */
Rotation<float> NineAxisIMU::GetEulerOrientation()
{
  Rotation<float> rotation;

  uint8_t buffer[6];
  rotation.status = ReadBytes(hi2c, i2c_address, eul_data, buffer, 6);
  rotation.heading = ((float)ConvertToSigned(Join(buffer[0], buffer[1]))) / eul_lsb_per_unit;
  rotation.roll = ((float)ConvertToSigned(Join(buffer[2], buffer[3]))) / eul_lsb_per_unit;
  rotation.pitch = ((float)ConvertToSigned(Join(buffer[4], buffer[5]))) / eul_lsb_per_unit;

  return rotation;
}

/* Returns the measured orientation in quarternion units */
Quarternion<float> NineAxisIMU::GetQuarternionOrientation()
{
  Quarternion<float> quarternion;

  uint8_t buffer[8];
  quarternion.status = ReadBytes(hi2c, i2c_address, qua_data, buffer, 8);
  quarternion.w = ((float)ConvertToSigned(Join(buffer[0], buffer[1]))) / qua_lsb_per_unit;
  quarternion.x = ((float)ConvertToSigned(Join(buffer[2], buffer[3]))) / qua_lsb_per_unit;
  quarternion.y = ((float)ConvertToSigned(Join(buffer[4], buffer[5]))) / qua_lsb_per_unit;
  quarternion.z = ((float)ConvertToSigned(Join(buffer[6], buffer[7]))) / qua_lsb_per_unit;

  return quarternion;
}

/* Returns a measurement of a 3d vector whose data addresses are ordered in sequence
 * of x_lsb, x_msb, y_lsb, y_msb, z_lsb, z_msb, starting at register_address.
 */
Vector3d<float> NineAxisIMU::Get3dVector(const uint8_t register_address, float lsb_per_unit)
{
  Vector3d<float> vector;

  uint8_t buffer[6];
  vector.status = ReadBytes(hi2c, i2c_address, register_address, buffer, 6);
  vector.x = ((float)ConvertToSigned(Join(buffer[0], buffer[1]))) / lsb_per_unit;
  vector.y = ((float)ConvertToSigned(Join(buffer[2], buffer[3]))) / lsb_per_unit;
  vector.z = ((float)ConvertToSigned(Join(buffer[4], buffer[5]))) / lsb_per_unit;
  return vector;
}

/* Take a 16 bit value where least 8 least significant and 8 most significant
 * are separated into separate variables and join them into one 16 bit variable.
 */
uint16_t NineAxisIMU::Join(uint8_t lsb, uint8_t msb)
{
  return lsb + (msb << 8);
}

/* Convert a 16 bit value in 2's complement form to signed int */
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

/* Convert a 8 bit value in 2's complement form to signed int */
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

} /* namespace sensor_interface */
} /* namespace stm32_code */




