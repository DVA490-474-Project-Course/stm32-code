/*
 * RGB_APDS_9960.h
 *
 *  Created on: Nov 12, 2024
 *      Author: Mudar Ibrahim
 */


#ifndef APDS9960_DRIVER_H
#define APDS9960_DRIVER_H

/* stm32h7xx_hal.h */
#include "stm32h7xx_hal.h"
/* common_types.h*/
#include "../common_types.h"

/* stm32_code*/
namespace stm32_code
{
/* sensor_drivers */
namespace sensor_drivers
{
/*!
 * @brief Initializes the APDS9960 sensor by enabling power and proximity detection.
 *
 * Long explanation about what this function does, how it interacts with the hardware,
 * and any specific notes about usage.
 *
 * @param[in] hi2c Handle to the I2C peripheral to communicate with the sensor.
 * @return Status Indicates whether the operation was successful or encountered an error.
 *
 * @note This function should be called after the I2C peripheral has been initialized.
 *
 * @see Other related functions or classes, if applicable.
 *
 * @pre Ensure that the I2C bus is correctly configured and that the APDS9960 sensor is
 * connected and powered.
 *
 * @warning Be mindful of the correct I2C address and potential issues with bus contention.
 */
Status InitializeAPDS9960(I2C_HandleTypeDef* hi2c);
/*!
 * @brief Reads proximity data from the APDS9960 sensor.
 *
 * This function reads the proximity data from the PDATA register of the APDS9960 sensor
 * over the I2C bus. The data read provides information about the detected proximity levels,
 * allowing further processing or decision-making based on the proximity value.
 *
 * @param[in] hi2c Handle to the I2C peripheral used for communication with the APDS9960 sensor.
 * @param[out] proximity_data Pointer to a variable where the read proximity value will be stored.
 *
 * @return Status Indicates whether the read operation was successful or encountered an error:
 * - Status::kOk if the data was read successfully.
 * - Status::kNotOk if there was an error during communication.
 *
 * @note The I2C peripheral must be properly initialized and configured before calling this function.
 * @note This function reads from the PDATA register (address `0x9C`) of the APDS9960 sensor.
 *
 * @see InitializeAPDS9960 for initializing the APDS9960 sensor.
 *
 * @pre Ensure that:
 * - The APDS9960 sensor has been initialized and enabled for proximity sensing.
 * - The I2C bus is correctly configured.
 *
 * @warning Incorrect I2C configuration or improper initialization of the APDS9960 sensor
 * may lead to communication errors or invalid proximity data.
 */
Status ReadProximityData(I2C_HandleTypeDef* hi2c, uint8_t* proximity_data);

} /* namespace sensor_drivers */
} /* namespace stm32_code */

#endif APDS9960_DRIVER_H /* PROJECTSTRUCTURE_MODULEA_A_H_ */
