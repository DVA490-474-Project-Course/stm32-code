/* nine_axis_imu.cc
 *==============================================================================
 * Author: Emil Åberg
 * Creation date: 2024-11-11
 * Last modified: 2024-12-13 by Emil Åberg
 * Description: Driver for the nine axis BNO055 IMU.
 * License: See LICENSE file for license details.
 *==============================================================================
 */

/* Related .h file */
#include "../../user-code/sensor-interface/nine_axis_imu.h"

/* Project .h files */
#include "stm32h7xx_hal.h"
#include "../../user-code/common_types.h"
#include "../../user-code/sensor-interface/read_write_functions.h"

namespace stm32_code
{
namespace sensor_interface
{

/* Default constructor */
NineAxisImu::NineAxisImu() {}

/* Initialise the nine axis IMU */
Status NineAxisImu::Init(I2C_HandleTypeDef* i2c_handle)
{
  Status status;

  this->i2c_handle = i2c_handle;
  status = WriteByte(i2c_handle, i2c_address, opr_mode, ndof);
  HAL_Delay(50);
  return status;
}

/* Returns the measured temperature in degrees celsius */
Scalar<uint8_t> NineAxisImu::GetTemperature()
{
  Scalar<uint8_t> temperature;
  uint8_t value;

  temperature.status = ReadByte(i2c_handle, i2c_address, temp_data, &value);
  temperature.value = ConvertToSignedInt8(value);
  return temperature;
}

/* Returns the measured magnetic field in micro tesla */
Vector3d<float> NineAxisImu::GetMagneticField()
{
  return Get3dVector(mag_data, mag_lsb_per_unit);
}

/* Returns the measured acceleration in m/s², acceleration due to gravity will
 * be included */
Vector3d<float> NineAxisImu::GetAcceleration()
{
  return Get3dVector(acc_data, acc_lsb_per_unit);
}

/* Returns the measured rotational speed in degrees/s */
Vector3d<float> NineAxisImu::GetRotationalSpeed()
{
  return Get3dVector(gyr_data, gyr_lsb_per_unit);
}

/* Returns the measured acceleration in m/s², acceleration due to gravity is
 * compensated for and will not be included */
Vector3d<float> NineAxisImu::GetLinearAcceleration()
{
  return Get3dVector(lia_data, lia_lsb_per_unit);
}

/* Returns the measured gravity vector in m/s² */
Vector3d<float> NineAxisImu::GetGravity()
{
  return Get3dVector(grv_data, grv_lsb_per_unit);
}

/* Returns the measured orientation in degrees */
Rotation<float> NineAxisImu::GetEulerOrientation()
{
  Rotation<float> rotation;

  uint8_t buffer[6];
  rotation.status = ReadBytes(i2c_handle, i2c_address, eul_data, buffer, 6);
  rotation.heading = ((float)ConvertToSignedInt16(Join(buffer[0], buffer[1]))) /
      eul_lsb_per_unit;
  rotation.roll = ((float)ConvertToSignedInt16(Join(buffer[2], buffer[3]))) /
      eul_lsb_per_unit;
  rotation.pitch = ((float)ConvertToSignedInt16(Join(buffer[4], buffer[5]))) /
      eul_lsb_per_unit;

  return rotation;
}

/* Returns the measured orientation in quarternion units */
Quarternion<float> NineAxisImu::GetQuarternionOrientation()
{
  Quarternion<float> quarternion;

  uint8_t buffer[8];
  quarternion.status = ReadBytes(i2c_handle, i2c_address, qua_data, buffer, 8);
  quarternion.w = ((float)ConvertToSignedInt16(Join(buffer[0], buffer[1]))) /
      qua_lsb_per_unit;
  quarternion.x = ((float)ConvertToSignedInt16(Join(buffer[2], buffer[3]))) /
      qua_lsb_per_unit;
  quarternion.y = ((float)ConvertToSignedInt16(Join(buffer[4], buffer[5]))) /
      qua_lsb_per_unit;
  quarternion.z = ((float)ConvertToSignedInt16(Join(buffer[6], buffer[7]))) /
      qua_lsb_per_unit;

  return quarternion;
}

/* Returns a measurement of a 3d vector whose data addresses are ordered in
 * sequence of x_lsb, x_msb, y_lsb, y_msb, z_lsb, z_msb, starting at
 * register_address.
 */
Vector3d<float> NineAxisImu::Get3dVector(const uint8_t register_address,
    float lsb_per_unit)
{
  Vector3d<float> vector;

  uint8_t buffer[6];
  vector.status = ReadBytes(i2c_handle, i2c_address, register_address, buffer, 6);
  vector.x = ((float)ConvertToSignedInt16(Join(buffer[0], buffer[1]))) /
      lsb_per_unit;
  vector.y = ((float)ConvertToSignedInt16(Join(buffer[2], buffer[3]))) /
      lsb_per_unit;
  vector.z = ((float)ConvertToSignedInt16(Join(buffer[4], buffer[5]))) /
      lsb_per_unit;
  return vector;
}

/* Take a 16 bit value where 8 least and 8 most significant bits are
 * separated and join them into one 16 bit variable.
 */
uint16_t NineAxisImu::Join(uint8_t lsb, uint8_t msb)
{
  return lsb + (msb << 8);
}

/* Convert a 16 bit value in 2's complement form to signed int */
int16_t NineAxisImu::ConvertToSignedInt16(uint16_t value)
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
int8_t NineAxisImu::ConvertToSignedInt8(uint8_t value)
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




