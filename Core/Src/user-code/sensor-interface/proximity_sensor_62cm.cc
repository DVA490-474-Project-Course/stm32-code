/* proximity_sensor_62cm.cc
 *==============================================================================
 * Author: Emil Åberg
 * Creation date: 2024-11-11
 * Last modified: 2024-11-20 by Emil Åberg
 * Description: Driver for the vl6180 62cm proximity sensor. This driver
 * functions as a wrapper for ST's
 * License: See LICENSE file for license details.
 *==============================================================================
 */

/* Related .h file */
#include "../sensor-interface/proximity_sensor_62cm.h"

/* Project .h files */
#include "../common_types.h"
#include "../stm32h7xx_hal.h"
#include "../../../Drivers/Adafruit_VL6180X/Adafruit_VL6180X.h"

namespace stm32_code
{
namespace sensor_interface
{

ProximitySensor62cm::ProximitySensor62cm() {}

Status ProximitySensor62cm::Init(I2C_HandleTypeDef* hi2c)
{
  bool status;

  vl = Adafruit_VL6180X(i2c_address);
  status = vl.begin(hi2c);

  if (status == true)
  {
    return Status::kOk;
  }
  else
  {
	return Status::kNotOk;
  }
}

Scalar<uint8_t> ProximitySensor62cm::GetDistance()
{
  Scalar<uint8_t> distance;
  uint8_t status;

  status = vl.readRangeStatus();
  distance.value = vl.readRange();

  if (status == VL6180X_ERROR_NONE)
  {
	distance.status = Status::kOk;
  }
  else
  {
    distance.status = Status::kNotOk;
  }

  return distance;
}

Scalar<float> ProximitySensor62cm::GetIlluminance()
{
  Scalar<float> luminosity;
  uint8_t status;

  status = vl.readRangeStatus();
  luminosity.value = vl.readLux(VL6180X_ALS_GAIN_5);

  if (status == VL6180X_ERROR_NONE)
  {
	luminosity.status = Status::kOk;
  }
  else
  {
	luminosity.status = Status::kNotOk;
  }

  return luminosity;
}

} /* namespace sensor_interface */
} /* namespace stm32_code */
