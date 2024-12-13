/* proximity_sensor_62cm.h
 *==============================================================================
 * Author: Emil Åberg
 * Creation date: 2024-11-11
 * Last modified: 2024-11-20 by Emil Åberg
 * Description: Driver for the VL6180X 62cm proximity sensor. This driver
 * functions as a wrapper for Adafruit's VL6180X Driver which can be found
 * here: https://github.com/adafruit/Adafruit_VL6180X
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

/* Default constructor */
ProximitySensor62cm::ProximitySensor62cm() {}


/* Configure the sensor to begin taking measurements */
Status ProximitySensor62cm::Init(I2C_HandleTypeDef* i2c_handle)
{
  bool status;

  vl = Adafruit_VL6180X(i2c_address);
  status = vl.begin(i2c_handle);

  if (status == true)
  {
    return Status::kOk;
  }
  else
  {
	return Status::kNotOk;
  }
}

/* Returns the measured distance in mm */
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

/* Returns the measured illuminence in lux */
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
