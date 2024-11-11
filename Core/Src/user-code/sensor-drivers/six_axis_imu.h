/* nine_axis_imu.h
 *==============================================================================
 * Author: Emil Åberg
 * Creation date: 2024-11-11
 * Last modified: 2024-11-11 by Emil Åberg
 * Description: Driver for the nine axis BNO055 IMU.
 * License: See LICENSE file for license details.
 *==============================================================================
 */

#ifndef SENSORDRIVERS_SIXAXISIMU_H
#define SENSORDRIVERS_SIXAXISIMU_H

/* Project .h files */
#include "../stm32h7xx_hal.h"

namespace sensor_drivers
{

class SixeAxisIMU
{
public:
	SixeAxisIMU();
  virtual ~SixeAxisIMU();
  static void PrintData(UART_HandleTypeDef* huart, MeasurementData measurement_data);
  MeasurementData ReadData(I2C_HandleTypeDef* hi2c);

private:
  int ReadData(I2C_HandleTypeDef* hi2c, uint8_t* buffer);
  MeasurementData JoinData(uint8_t* raw_data);
  uint16_t JoinData(uint8_t* raw_data, const uint8_t lsb_location, const uint8_t msb_location);

  /* IMU address */
  static const uint8_t imu_9axis_addr = 0x28;

  /* Data register addresses */
  static const uint8_t data_reg_first = 0x08;		/* Address of first data register to read */
  static const uint8_t data_reg_last = 0x34;		/* Address of last data register to read */

  /* Data register offsets, relative to data_reg_first */


  /* Size of all measurement data on sensor, in bytes */
  static const int data_size = data_reg_last - data_reg_first + 1;
};

} /* namespace sensor_drivers */

#endif /* SENSORDRIVERS_SIXAXISIMU_H */

