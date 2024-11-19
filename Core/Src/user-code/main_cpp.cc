/* main_cpp.cc
 *==============================================================================
 * Author: Emil Åberg & Mudar Ibrahim
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
#include "sensor-drivers/six_axis_imu.h"
#include "sensor-drivers/nine_axis_imu.h"
#include "sensor-drivers/proximity_sensor_120cm.h"
#include "sensor-drivers/read_write_functions.h"
#include "../stm32h7xx_hal.h"
#include "sensor-drivers/RGB_APDS_9960.h"
#include "stm32h7xx_hal.h"
#include "common_types.h"



/* Declare all necessary peripheral handles as extern */
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c4;
extern I2C_HandleTypeDef hi2c5;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;

extern "C"
/* stm32_code */
namespace stm32_code
{
/* sensor_drivers */
namespace sensor_drivers
{

/*
 * @brief Main function for user-defined operations in C++.
 *
 * This function initializes the APDS9960 proximity sensor via the I2C1
 * interface. It continuously reads proximity data from the sensor, transmits
 * the results over UART3, and handles potential communication errors. If the
 * sensor initialization fails, an error message is sent over UART3.
 *
 * @details
 * - The function starts by initializing I2C1 for sensor communication.
 * - It then attempts to initialize the APDS9960 sensor.
 * - Upon successful initialization, it enters an infinite loop where it reads
 *   proximity data and transmits the results.
 * - If the reading or initialization fails, appropriate error messages are
 *   transmitted via UART3.
 */
void MainCpp()
{
	// Initialize I2C1 interface
	 HAL_I2C_Init(&hi2c1);
	// Initialize the APDS9960 sensor
	  Status status = stm32_code::sensor_drivers::InitializeAPDS9960(&hi2c1);

	  if (status == Status::kOk)
	  {
		 uint8_t proximity_data = 0;
		 while (1)
		 {
			 // Read proximity data
			  status = stm32_code::sensor_drivers::ReadProximityData(&hi2c1, &proximity_data);

			  if (status == Status::kOk)
			      {
			        // Successfully read data, print it via UART
			        char buffer[100];
			        sprintf(buffer, "Proximity Data: %d\r\n", proximity_data);
			        HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
			      }
			  else
			        {
			          // Failed to read data, print error message via UART
			          char buffer[100];
			          sprintf(buffer, "Failed to read proximity data\r\n");
			          HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
			        }
			  HAL_Delay(500); // Delay for next read
		 }

	  }
	  else
	  {
		//  Initialization failed, print error message
		  char buffer[100];
		  sprintf(buffer, "Initialization failed\r\n");
		  HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
	  }

} /* namespace sensor_drivers */
} /* namespace stm32_code */

}/* extern "C" */

