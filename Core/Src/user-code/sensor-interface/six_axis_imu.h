/* nine_axis_imu.h
 *==============================================================================
 * Author: Emil Åberg
 * Creation date: 2024-11-11
 * Last modified: 2024-11-11 by Emil Åberg
 * Description: Driver for the six axis WSEN-ISDS IMU.
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

class SixAxisIMU
{
public:
  SixAxisIMU();
  virtual ~SixAxisIMU();
  Status Init(I2C_HandleTypeDef* hi2c);
  Vector3d<float> GetAcceleration();
  Vector3d<float> GetAngularAcceleration();
  Scalar<float> GetTemperature();

private:
  WE_sensorInterface_t isds;
  uint8_t address = 0x6b;
};

} /* namespace sensor_interface */
} /* namespace stm32_code */

#endif /* SENSORDRIVERS_SIXAXISIMU_H */

