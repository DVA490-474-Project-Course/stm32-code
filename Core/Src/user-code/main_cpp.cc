/* main_cpp.cc
 *==============================================================================
 * Author: Emil Åberg
 * Creation date: 2024-11-11
 * Last modified: 2024-11-11 by Emil Åberg
 * Description: Function that is called within main.c and serves as the main
 * entry point of the  user defined c++ code.S
 * License: See LICENSE file for license details.
 *==============================================================================
 */

/* Related .h files */
#include "main_cpp.h"

/* Project .h files */
#include "sensor-drivers/nine_axis_imu.h"
#include "sensor-drivers/proximity_sensor_120cm.h"
#include "../stm32h7xx_hal.h"

/* Declare all necessary peripheral handles as extern */
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c4;
extern I2C_HandleTypeDef hi2c5;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;

extern "C"
{

/* This function serves as the main entry point for the user defined c++ code */
void MainCpp()
{
  /* Example program to read sensor data via i2c and print it from the PC */
  stm32_code::sensor_drivers::ProximitySensor120cm proximity_sensor;
  uint8_t buffer[128];
  proximity_sensor.Init(&hi2c1);

  while(1)
  {
	proximity_sensor.ReadData(&hi2c1);
	if (proximity_sensor.IsDataValid())
	{
	  sprintf((char*)buffer, "Distance: %imm\r\n", proximity_sensor.GetDistance());
	  HAL_UART_Transmit(&huart3, buffer, strlen((char*)buffer), HAL_MAX_DELAY);
	}
  }
}

}
