/* nine_axis_imu.c
 *==============================================================================
 * Author: Emil Åberg
 * Creation date: 2024-11-11
 * Last modified: 2024-11-11 by Emil Åberg
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

MeasurementData NineAxisIMU::ReadData(I2C_HandleTypeDef* hi2c)
{
  uint8_t buffer[data_size];
  MeasurementData measurement_data;
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

int NineAxisIMU::ReadData(I2C_HandleTypeDef* hi2c, uint8_t* buffer)
{
  HAL_StatusTypeDef ret;

  buffer[0] = data_reg_first;
  ret = HAL_I2C_Master_Transmit(hi2c, imu_9axis_addr, buffer, 1, HAL_MAX_DELAY);

  if (ret != HAL_OK)
  {
    /* Error handling */
  }
  else
  {
    ret = HAL_I2C_Master_Receive(hi2c, imu_9axis_addr, buffer, data_size, HAL_MAX_DELAY);

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

uint16_t NineAxisIMU::JoinData(uint8_t* raw_data, const uint8_t lsb_location, const uint8_t msb_location)
{
  uint16_t lsb = raw_data[lsb_location];
  uint16_t msb = raw_data[msb_location];

  return lsb + (msb << 8);
}

MeasurementData NineAxisIMU::JoinData(uint8_t* raw_data)
{
  MeasurementData measurement_data;

  measurement_data.acc_data.x = JoinData(raw_data, acc_data_x_lsb, acc_data_x_msb);
  measurement_data.acc_data.y = JoinData(raw_data, acc_data_y_lsb, acc_data_y_msb);
  measurement_data.acc_data.z = JoinData(raw_data, acc_data_z_lsb, acc_data_z_msb);
  measurement_data.mag_data.x = JoinData(raw_data, mag_data_x_lsb, mag_data_x_msb);
  measurement_data.mag_data.y = JoinData(raw_data, mag_data_y_lsb, mag_data_y_msb);
  measurement_data.mag_data.z = JoinData(raw_data, mag_data_z_lsb, mag_data_z_msb);
  measurement_data.gyr_data.x = JoinData(raw_data, gyr_data_x_lsb, gyr_data_x_msb);
  measurement_data.gyr_data.y = JoinData(raw_data, gyr_data_y_lsb, gyr_data_y_msb);
  measurement_data.gyr_data.z = JoinData(raw_data, gyr_data_z_lsb, gyr_data_z_msb);
  measurement_data.eul_data.heading = JoinData(raw_data, eul_heading_lsb, eul_heading_msb);
  measurement_data.eul_data.roll = JoinData(raw_data, eul_roll_lsb, eul_roll_msb);
  measurement_data.eul_data.pitch = JoinData(raw_data, eul_pitch_lsb, eul_pitch_msb);
  measurement_data.qua_data.w = JoinData(raw_data, qua_data_w_lsb, qua_data_w_msb);
  measurement_data.qua_data.x = JoinData(raw_data, qua_data_x_lsb, qua_data_x_msb);
  measurement_data.qua_data.y = JoinData(raw_data, qua_data_y_lsb, qua_data_y_msb);
  measurement_data.qua_data.z = JoinData(raw_data, qua_data_z_lsb, qua_data_z_msb);
  measurement_data.lia_data.x = JoinData(raw_data, lia_data_x_lsb, lia_data_x_msb);
  measurement_data.lia_data.y = JoinData(raw_data, lia_data_y_lsb, lia_data_y_msb);
  measurement_data.lia_data.z = JoinData(raw_data, lia_data_z_lsb, lia_data_z_msb);
  measurement_data.grv_data.x = JoinData(raw_data, grv_data_x_lsb, grv_data_x_msb);
  measurement_data.grv_data.y = JoinData(raw_data, grv_data_y_lsb, grv_data_y_msb);
  measurement_data.grv_data.z = JoinData(raw_data, grv_data_z_lsb, grv_data_z_msb);
  measurement_data.temp = raw_data[temp];

  return measurement_data;
}

void NineAxisIMU::PrintData(UART_HandleTypeDef* huart, MeasurementData measurement_data)
{
  uint8_t buffer[128];
  sprintf((char*)buffer, "Acceleration: (%i, %i, %i)\r\n",
		  measurement_data.acc_data.x,
		  measurement_data.acc_data.y,
		  measurement_data.acc_data.z);
  HAL_UART_Transmit(huart, buffer, strlen((char*)buffer), HAL_MAX_DELAY);
  sprintf((char*)buffer, "Magnetic field: (%i, %i, %i)\r\n",
  		  measurement_data.mag_data.x,
  		  measurement_data.mag_data.y,
  		  measurement_data.mag_data.z);
  HAL_UART_Transmit(huart, buffer, strlen((char*)buffer), HAL_MAX_DELAY);
  sprintf((char*)buffer, "Gyroscope: (%i, %i, %i)\r\n",
		  measurement_data.gyr_data.x,
		  measurement_data.gyr_data.y,
		  measurement_data.gyr_data.z);
  HAL_UART_Transmit(huart, buffer, strlen((char*)buffer), HAL_MAX_DELAY);
  sprintf((char*)buffer, "Euler: (%i, %i, %i)\r\n",
  		  measurement_data.eul_data.heading,
  		  measurement_data.eul_data.roll,
  		  measurement_data.eul_data.pitch);
  HAL_UART_Transmit(huart, buffer, strlen((char*)buffer), HAL_MAX_DELAY);
  sprintf((char*)buffer, "Quarternion: (%i, %i, %i, %i)\r\n",
		  measurement_data.qua_data.w,
		  measurement_data.qua_data.x,
		  measurement_data.qua_data.y,
		  measurement_data.qua_data.z);
  HAL_UART_Transmit(huart, buffer, strlen((char*)buffer), HAL_MAX_DELAY);
  sprintf((char*)buffer, "Linear acceleration: (%i, %i, %i)\r\n",
		  measurement_data.lia_data.x,
		  measurement_data.lia_data.y,
		  measurement_data.lia_data.z);
  HAL_UART_Transmit(huart, buffer, strlen((char*)buffer), HAL_MAX_DELAY);
  sprintf((char*)buffer, "Gravity: (%i, %i, %i)\r\n",
  		  measurement_data.grv_data.x,
  		  measurement_data.grv_data.y,
  		  measurement_data.grv_data.z);
  HAL_UART_Transmit(huart, buffer, strlen((char*)buffer), HAL_MAX_DELAY);
  sprintf((char*)buffer, "Temperature: %i\r\n", measurement_data.temp);
  HAL_UART_Transmit(huart, buffer, strlen((char*)buffer), HAL_MAX_DELAY);
}

} /* namespace sensor_drivers */
} /* namespace stm32_code */




