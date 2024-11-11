/* main_cpp.cc
 *==============================================================================
 * Author: Emil Åberg
 * Creation date: 2024-11-11
 * Last modified: 2024-11-11 by Emil Åberg
 * Description: Function that is called within main.c and serves as the main
 * entry point of the  user defined c++ code.
 * License: See LICENSE file for license details.
 *==============================================================================
 */

/* Related .h files */
#include "main_cpp.h"

/* Project .h files */
#include "sensor-drivers/nine_axis_imu.h"
#include "../stm32h7xx_hal.h"

extern "C"
{

/* This function serves as the main entry point for the user defined c++ code.
 * All necessary peripheral handles should be added as arguments. */
void MainCpp(I2C_HandleTypeDef hi2c1,
			 I2C_HandleTypeDef hi2c2,
			 I2C_HandleTypeDef hi2c4,
			 I2C_HandleTypeDef hi2c5,
			 UART_HandleTypeDef huart3,
			 UART_HandleTypeDef huart5)
{
  /* Example program to read sensor data via i2c and print it from the PC */
  stm32_code::sensor_drivers::NineAxisIMU imu;
  stm32_code::sensor_drivers::NineAxisIMUData data;

  while(1)
  {
	data = imu.ReadData(&hi2c2);
	imu.PrintData(&huart3, data);
  }
}

}
