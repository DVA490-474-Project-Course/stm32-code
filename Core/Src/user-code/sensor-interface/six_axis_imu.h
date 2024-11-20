/* nine_axis_imu.h
 *==============================================================================
 * Author: Emil Åberg
 * Creation date: 2024-11-11
 * Last modified: 2024-11-20 by Emil Åberg
 * Description: Driver for the six axis WSEN_ISDS IMU. This driver
 * functions as a wrapper for Wurth Elektronik's WSEN_ISDS driver which can be
 * found here:
 * https://github.com/WurthElektronik/Sensors-SDK_STM32/tree/main/SensorsSDK/WSEN_ISDS_2536030320001
 * License: See LICENSE file for license details.
 *==============================================================================
 */

#ifndef SENSORDRIVERS_SIXAXISIMU_H
#define SENSORDRIVERS_SIXAXISIMU_H

/* Project .h files */
#include "../stm32h7xx_hal.h"
#include "../common_types.h"
#include "../../../Drivers/WSEN_ISDS_2536030320001/WSEN_ISDS_2536030320001.h"

namespace stm32_code
{
namespace sensor_interface
{

/*!
 * @brief Class representing driver control of one WSEN_ISDS six axis imu.
 *
 * Class representing driver control of one WSEN_ISDS six axis imu.
 */
class SixAxisIMU
{
public:
  /*!
   * @brief Default constructor
   *
   * The default constructor.
   */
  SixAxisIMU();

  /*!
   * @brief Initializes the sensor.
   *
   * Configure the sensor to begin taking measuremets.
   *
   * @param[in] hi2c Pointer to the handle of the I2C peripheral.
   * 
   * @return Enum indicating wheter initialization was successful.
   */
  Status Init(I2C_HandleTypeDef* hi2c);

  /*!
   * @brief Get the measured acceleration.
   *
   * Returns the measured acceleration in mg.
   *
   * @pre Sensor has been initialized with Init().
   *
   * @return Struct containing the acceleration vector and status
   * enum indicating whether sensor read was successful.
   */
  Vector3d<float> GetAcceleration();

  /*!
   * @brief Get the measured angular speed.
   *
   * Returns the measured angular speed in milli degrees/s.
   *
   * @pre Sensor has been initialized with Init().
   *
   * @return Struct containing the angular velocity vector and status
   * enum indicating whether sensor read was successful.
   */
  Vector3d<float> GetAngularSpeed();

  /*!
   * @brief Get the measured temperature.
   *
   * Returns the measured temperature in degrees Celsius.
   *
   * @pre Sensor has been initialized with Init().
   *
   * @return Struct containing temperature value and status enum indicating
   * whether sensor read was successful.
   */
  Scalar<float> GetTemperature();

private:
  WE_sensorInterface_t isds;
  uint8_t address = 0x6b;
};

} /* namespace sensor_interface */
} /* namespace stm32_code */

#endif /* SENSORDRIVERS_SIXAXISIMU_H */

