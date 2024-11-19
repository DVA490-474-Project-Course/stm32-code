/* RGB_APDS_9960.CC
 *==============================================================================
 * Author: Mudar Ibrahim
 * Creation date: 2024-11-12
 * Last modified: 2024-11-19 by Mudar Ibrahim
 * Description: This code defines a driver for the APDS9960 sensor,
 * which is an advanced proximity and RGB sensor used for various applications,
 * including proximity detection and ambient light sensing.
 * The driver is designed for use on an STM32 microcontroller with the HAL library
 * for I2C communication.
 * The code implements functions to initialize the APDS9960 sensor
 * by enabling its power and proximity detection capabilities and to read proximity data from the sensor.
 *
 * License: See LICENSE file for license details.
 *==============================================================================
 */

/*RGB_APDS_9960.h */
#include "RGB_APDS_9960.h"
/*read_write_functions.h*/
#include "read_write_functions.h"
/*stdio.h '*/
#include <stdio.h>
/*stm32h7xx_hal.h*/
#include "stm32h7xx_hal.h"
/*common_types.h*/
#include "../common_types.h"
/*I2C address of the APDS9960*/
#define APDS9960_ADDR (0x39 << 1)
/*ENABLE register address*/
#define ENABLE_REG 0x80
/*Proximity data register address*/
#define PDATA_REG 0x9C

/*stm32_code*/
namespace stm32_code
{
/* sensor_drivers */
namespace sensor_drivers
{
/* Initializes the APDS9960 sensor by enabling power and proximity detection. */
Status InitializeAPDS9960(I2C_HandleTypeDef* hi2c)
{
  uint8_t enable_value = 0x05;  /* Enable power (PON) and proximity detection (PEN)*/

  /* Write the value to the ENABLE register to enable power and proximity detection*/
  return WriteByte(hi2c, APDS9960_ADDR, ENABLE_REG, enable_value);
  HAL_Delay(10);
}
/* Reads proximity data from the APDS9960 sensor.*/
Status ReadProximityData(I2C_HandleTypeDef* hi2c, uint8_t* proximity_data)
{
  /* Read proximity data from the PDATA register*/
  return ReadByte(hi2c, APDS9960_ADDR, PDATA_REG, proximity_data);
  HAL_Delay(10);
}

} /* namespace sensor_drivers */
} /* namespace stm32_code */
