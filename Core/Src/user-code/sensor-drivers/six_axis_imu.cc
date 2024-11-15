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
#include "../../../Drivers/WSEN_ISDS_2536030320001/WSEN_ISDS_2536030320001.h"
#include "../common_types.h"

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

Status SixAxisIMU::Init(I2C_HandleTypeDef* hi2c)
{
  uint8_t deviceIdValue = 0;

  ISDS_getDefaultInterface(&isds);
  isds.interfaceType = WE_i2c;
  isds.options.i2c.burstMode = 1;
  isds.options.i2c.address = address;
  isds.handle = hi2c;

  /* Wait for boot */
  HAL_Delay(50);

  /* Communication test */
  if (WE_SUCCESS != ISDS_getDeviceID(&isds, &deviceIdValue) ||
		  (deviceIdValue != ISDS_DEVICE_ID_VALUE))
  {
	  return Status::kNotOk;
  }

  /* Perform soft reset of the sensor */
  ISDS_softReset(&isds, ISDS_enable);
  ISDS_state_t swReset;
  do
  {
    ISDS_getSoftResetState(&isds, &swReset);
  }
  while (swReset);

  /* Perform reboot (retrieve trimming parameters from nonvolatile memory) */
  ISDS_reboot(&isds, ISDS_enable);
  HAL_Delay(15);

  /* Enable block data update */
  ISDS_enableBlockDataUpdate(&isds, ISDS_enable);

  /* Sampling rate (104 Hz) */
  ISDS_setAccOutputDataRate(&isds, ISDS_accOdr104Hz);
  ISDS_setGyroOutputDataRate(&isds, ISDS_gyroOdr104Hz);

  /* Accelerometer 2g range */
  ISDS_setAccFullScale(&isds, ISDS_accFullScaleTwoG);

  /* Gyroscope 2000 dps range */
  ISDS_setGyroFullScale(&isds, ISDS_gyroFullScale2000dps);

  return Status::kOk;
}

Vector3d<float> SixAxisIMU::GetAcceleration()
{
  ISDS_state_t dataReady;
  Vector3d<float> acceleration;

  /* Wait until the acceleration values are ready to read */
  do
  {
    ISDS_isAccelerationDataReady(&isds, &dataReady);
  }
  while (dataReady == ISDS_disable);

  /* Read acceleration values */
  if (ISDS_getAccelerations_float(&isds, &acceleration.x, &acceleration.y,
		  &acceleration.z) != WE_SUCCESS)
  {
    acceleration.status = Status::kNotOk;
  }
  else
  {
	acceleration.status = Status::kOk;
  }

  return acceleration;
}

Vector3d<float> SixAxisIMU::GetAngularAcceleration()
{
  ISDS_state_t dataReady;
  Vector3d<float> acceleration;

  /* Wait until the acceleration values are ready to read */
  do
  {
    ISDS_isGyroscopeDataReady(&isds, &dataReady);
  }
  while (dataReady == ISDS_disable);

  /* Read acceleration values */
  if (ISDS_getAngularRates_float(&isds, &acceleration.x, &acceleration.y,
		  &acceleration.z) != WE_SUCCESS)
  {
    acceleration.status = Status::kNotOk;
  }
  else
  {
	acceleration.status = Status::kOk;
  }

  return acceleration;
}

Scalar<float> SixAxisIMU::GetTemperature()
{
  ISDS_state_t dataReady;
  Scalar<float> temperature;

  /* Wait until the acceleration values are ready to read */
  do
  {
    ISDS_isTemperatureDataReady(&isds, &dataReady);
  }
  while (dataReady == ISDS_disable);

  /* Read temperature value */
  if (ISDS_getTemperature_float(&isds, &temperature.value) != WE_SUCCESS)
  {
    temperature.status = Status::kNotOk;
  }
  else
  {
    temperature.status = Status::kOk;
  }

  return temperature;
}


} /* namespace sensor_drivers */
} /* namespace stm32_code */




