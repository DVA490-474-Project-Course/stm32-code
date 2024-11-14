/* nine_axis_imu.h
 *==============================================================================
 * Author: Emil Åberg
 * Creation date: 2024-11-11
 * Last modified: 2024-11-11 by Emil Åberg
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

struct NineAxisIMUData
{
  Vector3d<uint16_t> acc_data;		/* m/s² */
  Vector3d<uint16_t> mag_data;		/* uT */
  Vector3d<uint16_t> gyr_data;		/* Dps */
  Rotation<uint16_t> eul_data;		/* degrees */
  Quarternion<uint16_t> qua_data;		/* quarternion units */
  Vector3d<uint16_t> lia_data;		/* m/s² */
  Vector3d<uint16_t> grv_data;		/* m/s² */
  uint8_t temp;				/* C */
};

class NineAxisIMU
{
public:
  NineAxisIMU();
  virtual ~NineAxisIMU();
  static void PrintData(UART_HandleTypeDef* huart, NineAxisIMUData measurement_data);
  NineAxisIMUData ReadData(I2C_HandleTypeDef* hi2c);

private:
  int ReadData(I2C_HandleTypeDef* hi2c, uint8_t* buffer);
  NineAxisIMUData JoinData(uint8_t* raw_data);
  uint16_t JoinData(uint8_t* raw_data, const uint8_t lsb_location, const uint8_t msb_location);

  /* IMU address */
  static const uint8_t imu_9axis_addr = 0x28;

  /* Data register addresses */
  static const uint8_t data_reg_first = 0x08;		/* Address of first data register to read */
  static const uint8_t data_reg_last = 0x34;		/* Address of last data register to read */

  /* Data register offsets, relative to data_reg_first */
  static const uint8_t acc_data_x_lsb = 0x00;
  static const uint8_t acc_data_x_msb = 0x01;
  static const uint8_t acc_data_y_lsb = 0x02;
  static const uint8_t acc_data_y_msb = 0x03;
  static const uint8_t acc_data_z_lsb = 0x04;
  static const uint8_t acc_data_z_msb = 0x05;
  static const uint8_t mag_data_x_lsb = 0x06;
  static const uint8_t mag_data_x_msb = 0x07;
  static const uint8_t mag_data_y_lsb = 0x08;
  static const uint8_t mag_data_y_msb = 0x09;
  static const uint8_t mag_data_z_lsb = 0x0a;
  static const uint8_t mag_data_z_msb = 0x0b;
  static const uint8_t gyr_data_x_lsb = 0x0c;
  static const uint8_t gyr_data_x_msb = 0x0d;
  static const uint8_t gyr_data_y_lsb = 0x0e;
  static const uint8_t gyr_data_y_msb = 0x0f;
  static const uint8_t gyr_data_z_lsb = 0x10;
  static const uint8_t gyr_data_z_msb = 0x11;
  static const uint8_t eul_heading_lsb = 0x12;
  static const uint8_t eul_heading_msb = 0x13;
  static const uint8_t eul_roll_lsb = 0x14;
  static const uint8_t eul_roll_msb = 0x15;
  static const uint8_t eul_pitch_lsb = 0x16;
  static const uint8_t eul_pitch_msb = 0x17;
  static const uint8_t qua_data_w_lsb = 0x18;
  static const uint8_t qua_data_w_msb = 0x19;
  static const uint8_t qua_data_x_lsb = 0x1a;
  static const uint8_t qua_data_x_msb = 0x1b;
  static const uint8_t qua_data_y_lsb = 0x1c;
  static const uint8_t qua_data_y_msb = 0x1d;
  static const uint8_t qua_data_z_lsb = 0x1e;
  static const uint8_t qua_data_z_msb = 0x1f;
  static const uint8_t lia_data_x_lsb = 0x20;
  static const uint8_t lia_data_x_msb = 0x21;
  static const uint8_t lia_data_y_lsb = 0x22;
  static const uint8_t lia_data_y_msb = 0x23;
  static const uint8_t lia_data_z_lsb = 0x24;
  static const uint8_t lia_data_z_msb = 0x25;
  static const uint8_t grv_data_x_lsb = 0x26;
  static const uint8_t grv_data_x_msb = 0x27;
  static const uint8_t grv_data_y_lsb = 0x28;
  static const uint8_t grv_data_y_msb = 0x29;
  static const uint8_t grv_data_z_lsb = 0x2a;
  static const uint8_t grv_data_z_msb = 0x2b;
  static const uint8_t temp = 0x2c;

  /* Size of all measurement data on sensor, in bytes */
  static const int data_size = data_reg_last - data_reg_first + 1;
};

} /* namespace sensor_drivers */
} /* namespace stm32_code */

#endif /* STM32CODE_SENSORDRIVERS_NINEAXISIMU_H */
