/* nine_axis_imu.h
 *==============================================================================
 * Author: Emil Åberg
 * Creation date: 2024-11-11
 * Last modified: 2024-11-15 by Emil Åberg
 * Description: Driver for the nine axis BNO055 IMU.
 * License: See LICENSE file for license details.
 *==============================================================================
 */

#ifndef STM32CODE_SENSORDRIVERS_NINEAXISIMU_H
#define STM32CODE_SENSORDRIVERS_NINEAXISIMU_H

/* Project .h files */
#include "../stm32h7xx_hal.h"
#include "../common_types.h"

namespace stm32_code
{
namespace sensor_drivers
{

class NineAxisIMU
{
public:
  NineAxisIMU();
  ~NineAxisIMU();
  Status Init(I2C_HandleTypeDef* hi2c);
  Scalar<uint8_t> GetTemperature();
  Vector3d<float> GetMagneticField();
  Vector3d<float> GetRotationalAcceleration();
  Vector3d<float> GetAcceleration();
  Vector3d<float> GetLinearAcceleration();
  Vector3d<float> GetGravity();
  Rotation<float> GetEulerOrientation();
  Quarternion<float> GetQuarternionOrientation();

private:
  Vector3d<float> Get3dVector(const uint8_t register_address, float factor);
  uint16_t Join(uint8_t lsb, uint8_t msb);
  int16_t ConvertToSigned(uint16_t value);
  int8_t ConvertToSigned(uint8_t value);
  I2C_HandleTypeDef* hi2c;

  /* Operating mode register */
  static const uint8_t opr_mode = 0x3d;

  /* "Nine Degrees Of Freedom" Operation mode; enables all sensors and sensor fusion */
  static const uint8_t ndof = 0b1100;

  /* IMU address */
  static const uint8_t i2c_address = 0x28 << 1;

  /* Data registers */
  static const uint8_t acc_data_x_lsb = 0x08;
  static const uint8_t mag_data_x_lsb = 0x0e;
  static const uint8_t gyr_data_x_lsb = 0x14;
  static const uint8_t eul_heading_lsb = 0x1a;
  static const uint8_t qua_data_w_lsb = 0x20;
  static const uint8_t lia_data_x_lsb = 0x28;
  static const uint8_t grv_data_x_lsb = 0x2e;
  static const uint8_t temp = 0x34;
};

} /* namespace sensor_drivers */
} /* namespace stm32_code */

#endif /* STM32CODE_SENSORDRIVERS_NINEAXISIMU_H */
