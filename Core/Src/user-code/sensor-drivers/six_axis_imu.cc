/* nine_axis_imu.c
 *==============================================================================
 * Author: Emil Åberg
 * Creation date: 2024-11-11
 * Last modified: 2024-11-11 by Emil Åberg
 * Description: Driver for the six axis WSEN-ISDS IMU.
 * License: See LICENSE file for license details.
 *==============================================================================
 */

/* Related .h files */
#include "six_axis_imu.h"

/* C++ standard library headers */
#include <string.h>
#include <stdio.h>

/* Project .h files */
#include "../stm32h7xx_hal.h"

namespace stm32_code
{
namespace sensor_drivers
{

SixAxisIMU::SixAxisIMU()
{

}

SixAxisIMU::~SixAxisIMU()
{

}

SixAxisIMUData SixAxisIMU::ReadData(I2C_HandleTypeDef* hi2c)
{
  uint8_t buffer[data_size];
  SixAxisIMUData measurement_data;
  int status;

  status = ReadData(hi2c, buffer);

  if (status != 0)
  {
	/* Error handling */
  }
  else
  {
	measurement_data = JoinData(buffer);
  }

  return measurement_data;
}

int SixAxisIMU::ReadData(I2C_HandleTypeDef* hi2c, uint8_t* buffer)
{
  HAL_StatusTypeDef ret;

  buffer[0] = data_reg_first;
  ret = HAL_I2C_Master_Transmit(hi2c, imu_addr, buffer, 1, HAL_MAX_DELAY);

  if (ret != HAL_OK)
  {
    /* Error handling */
  }
  else
  {
    ret = HAL_I2C_Master_Receive(hi2c, imu_addr, buffer, data_size, HAL_MAX_DELAY);

    if (ret != HAL_OK)
    {
      /* Error handling */
    }
    else
    {
      return 0;
    }
  }

  return -1;
}

uint16_t SixAxisIMU::JoinData(uint8_t* raw_data, const uint8_t lsb_location, const uint8_t msb_location)
{
  uint16_t lsb = raw_data[lsb_location];
  uint16_t msb = raw_data[msb_location];

  return lsb + (msb << 8);
}

SixAxisIMUData SixAxisIMU::JoinData(uint8_t* raw_data)
{
  SixAxisIMUData measurement_data;
  uint16_t t_out;

  t_out = JoinData(raw_data, t_out_l, t_out_h);

  /* Convert from 2s complement */
  if (t_out > 0x7FF)
  {
    t_out |= 0xF000;
  }

  /* Convert to float Celsius */
  measurement_data.temperature = t_out * temperature_factor + temperature_zero;

  /* Convert raw acceleration data to mg */
  measurement_data.acceleration.x = JoinData(raw_data, xl_x_out_l, xl_x_out_h) * accelerometer_factor;
  measurement_data.acceleration.y = JoinData(raw_data, xl_y_out_l, xl_y_out_h) * accelerometer_factor;
  measurement_data.acceleration.z = JoinData(raw_data, xl_z_out_l, xl_z_out_h) * accelerometer_factor;

  /* Convert raw gyroscope data to mdps */
  measurement_data.angular_accelarion.x = JoinData(raw_data, xl_x_out_l, xl_x_out_h) * gyroscope_factor;
  measurement_data.angular_accelarion.y = JoinData(raw_data, xl_y_out_l, xl_y_out_h) * gyroscope_factor;
  measurement_data.angular_accelarion.z = JoinData(raw_data, xl_z_out_l, xl_z_out_h) * gyroscope_factor;

  return measurement_data;
}

void SixAxisIMU::PrintData(UART_HandleTypeDef* huart, SixAxisIMUData measurement_data)
{
  uint8_t buffer[128];
  sprintf((char*)buffer, "Acceleration: (%f, %f, %f)\r\n",
		  measurement_data.acceleration.x,
		  measurement_data.acceleration.y,
		  measurement_data.acceleration.z);
  HAL_UART_Transmit(huart, buffer, strlen((char*)buffer), HAL_MAX_DELAY);
  sprintf((char*)buffer, "Magnetic field: (%i, %i, %i)\r\n",
		  measurement_data.angular_accelarion.x,
		  measurement_data.angular_accelarion.y,
		  measurement_data.angular_accelarion.z);
  HAL_UART_Transmit(huart, buffer, strlen((char*)buffer), HAL_MAX_DELAY);
  sprintf((char*)buffer, "Temperature: %f\r\n", measurement_data.temperature);
  HAL_UART_Transmit(huart, buffer, strlen((char*)buffer), HAL_MAX_DELAY);
}

} /* namespace sensor_drivers */
} /* namespace stm32_code */




