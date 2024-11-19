
#include "../sensor-interface/proximity_sensor_120cm.h"

#include "../stm32h7xx_hal.h"
#include "../../../Drivers/VL53L4CD_ULD_Driver/VL53L4CD_api.h"
#include "../common_types.h"

namespace stm32_code
{
namespace sensor_interface
{

ProximitySensor120cm::ProximitySensor120cm() {}

ProximitySensor120cm::~ProximitySensor120cm() {}

Status ProximitySensor120cm::Init(I2C_HandleTypeDef* hi2c)
{
  VL53L4CD_Error status;
  this->hi2c = hi2c;
  status = VL53L4CD_SensorInit(hi2c, address);

  if (status == 0)
  {
    status = VL53L4CD_StartRanging(hi2c, address);
  }

  if (status == 0)
  {
    return Status::kOk;
  }
  else
  {
	return Status::kNotOk;
  }
}

Scalar<uint16_t> ProximitySensor120cm::GetDistance()
{
  uint8_t data_ready;
  VL53L4CD_ResultsData_t result;
  Scalar<uint16_t> distance;

  VL53L4CD_CheckForDataReady(hi2c, address, &data_ready);

  if (data_ready == 1)
  {
	VL53L4CD_GetResult(hi2c, address, &result);
	VL53L4CD_ClearInterrupt(hi2c, address);
	distance.value = result.distance_mm;

	if (result.range_status == 0)
	{
	  distance.status = Status::kOk;
	}
	else
	{
      distance.status = Status::kNotOk;
	}
  }
  else
  {
    distance.status;
  }

  return distance;
}

} /* namespace sensor_interface */
} /* namespace stm32_code */
