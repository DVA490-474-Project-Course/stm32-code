/* nine_axis_imu.h
 *==============================================================================
 * Author: Emil Åberg
 * Creation date: 2024-11-11
 * Last modified: 2024-11-11 by Emil Åberg
 * Description: Driver for the six axis WSEN-ISDS IMU.
 * License: See LICENSE file for license details.
 *==============================================================================
 */

#ifndef SENSORDRIVERS_SIXAXISIMU_H
#define SENSORDRIVERS_SIXAXISIMU_H

/* Project .h files */
#include "../stm32h7xx_hal.h"
#include "../common_types.h"

namespace stm32_code
{
namespace sensor_drivers
{

struct SixAxisIMUData
{
  Vector_3d<float> acceleration; 			/* mg */
  Vector_3d<uint16_t> angular_accelarion;	/* mdps */
  float temperature;						/* C */
};

class SixAxisIMU
{
public:
  SixAxisIMU();
  virtual ~SixAxisIMU();
  static void PrintData(UART_HandleTypeDef* huart, SixAxisIMUData measurement_data);
  SixAxisIMUData ReadData(I2C_HandleTypeDef* hi2c);

private:
  int ReadData(I2C_HandleTypeDef* hi2c, uint8_t* buffer);
  SixAxisIMUData JoinData(uint8_t* raw_data);
  uint16_t JoinData(uint8_t* raw_data, const uint8_t lsb_location, const uint8_t msb_location);

  /* IMU address */
  static const uint8_t imu_addr = 0x6b << 1;

  /* Data register addresses */
  static const uint8_t data_reg_first = 0x20;		/* Address of first data register to read */
  static const uint8_t data_reg_last = 0x2d;		/* Address of last data register to read */

  /* Data register offsets, relative to data_reg_first */
  static const uint8_t t_out_l = 0x00;
  static const uint8_t t_out_h = 0x01;
  static const uint8_t g_x_out_l = 0x02;
  static const uint8_t g_x_out_h = 0x03;
  static const uint8_t g_y_out_l = 0x04;
  static const uint8_t g_y_out_h = 0x05;
  static const uint8_t g_z_out_l = 0x06;
  static const uint8_t g_z_out_h = 0x07;
  static const uint8_t xl_x_out_l = 0x08;
  static const uint8_t xl_x_out_h = 0x09;
  static const uint8_t xl_y_out_l = 0x0a;
  static const uint8_t xl_y_out_h = 0x0b;
  static const uint8_t xl_z_out_l = 0x0c;
  static const uint8_t xl_z_out_h = 0x0d;

  /* Other registers */
  static const uint8_t ctrl1_xl = 0x10;
  static const uint8_t ctrl2_g = 0x11;
  static const uint8_t ctrl3_c = 0x12;
  static const uint8_t ctrl6_c = 0x15;
  static const uint8_t ctrl7_g = 0x16;
  static const uint8_t ctrl8_xl = 0x17;

  /* Size of all measurement data on sensor, in bytes */
  static const int data_size = data_reg_last - data_reg_first + 1;

  /* Acceleration data received should be multiplied by this value in order
   * to convert it to mg. */
  static constexpr float accelerometer_factor = 0.488F;

  /* Temperature data received needs to be multiplied by this calue in order
   * to convert to celsius.
   */
  static constexpr float temperature_factor = 1.0F / 256.0F;

  static constexpr float temperature_zero = 25.0F;

  /* Gyroscope data received should be multiplied by this value in order
     * to convert it to mg. */
  static const int gyroscope_factor = 70;
};

} /* namespace sensor_drivers */
} /* namespace stm32_code */

#endif /* SENSORDRIVERS_SIXAXISIMU_H */

