/*
 * read_write_functions.h
 *
 *  Created on: Nov 15, 2024
 *      Author: vicore
 */

#ifndef STM32CODE_SENSORDRIVERS_READWRITE_FUNCTIONS_H
#define STM32CODE_SENSORDRIVERS_READWRITE_FUNCTIONS_H

#include "../stm32h7xx_hal.h"
#include "../common_types.h"

namespace stm32_code
{
namespace sensor_drivers
{

Status ReadByte(I2C_HandleTypeDef* hi2c, uint8_t i2c_address, uint8_t register_address, uint8_t* value);
Status ReadBytes(I2C_HandleTypeDef* hi2c, uint8_t i2c_address, uint8_t register_address, uint8_t buffer[], int buffer_size);
Status WriteByte(I2C_HandleTypeDef* hi2c, uint8_t i2c_address, uint8_t register_address, uint8_t value);

} /* namespace sensor_drivers */
} /* namespace stm32_code */

#endif /* STM32CODE_SENSORDRIVERS_READWRITE_FUNCTIONS_H */
