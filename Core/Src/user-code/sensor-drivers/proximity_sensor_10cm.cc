
#include "proximity_sensor_10cm.h"
#include "../stm32h7xx_hal.h"
#include "../../../Drivers/VL53L4CD_ULD_Driver/VL53L4CD_api.h"
#include "../common_types.h"

namespace stm32_code
{
namespace sensor_drivers
{

ProximitySensor10cm::ProximitySensor10cm() {}

ProximitySensor10cm::~ProximitySensor10cm() {}

Status ProximitySensor10cm::Init(I2C_HandleTypeDef* hi2c)
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

Scalar<uint8_t> ProximitySensor10cm::GetDistance()
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

Scalar<float> ProximitySensor10cm::GetIlluminance()
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

} /* namespace sensor_drivers */
} /* namespace stm32_code */
