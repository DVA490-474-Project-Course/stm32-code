
#include "proximity_sensor_120cm.h"
#include "../stm32h7xx_hal.h"
#include "../../../Drivers/VL53L4CD_ULD_Driver/VL53L4CD_api.h"

namespace stm32_code
{
namespace sensor_drivers
{

ProximitySensor120cm::ProximitySensor120cm()
{
  data_valid = false;
  distance_mm = 0;
}

ProximitySensor120cm::~ProximitySensor120cm()
{

}

int ProximitySensor120cm::Init(I2C_HandleTypeDef* hi2c)
{
  VL53L4CD_Error status;
  status = VL53L4CD_SensorInit(hi2c, address);

  if (status == 0)
  {
    status = VL53L4CD_StartRanging(hi2c, address);
  }

  if (status == 0)
  {
    return 0;
  }
  else
  {
    return -1;
  }
}

int ProximitySensor120cm::ReadData(I2C_HandleTypeDef* hi2c)
{
  uint8_t data_ready;
  VL53L4CD_Error status;
  VL53L4CD_ResultsData_t result;

  VL53L4CD_CheckForDataReady(hi2c, address, &data_ready);

  if (data_ready == 1)
  {
	VL53L4CD_GetResult(hi2c, address, &result);
    VL53L4CD_ClearInterrupt(hi2c, address);
    distance_mm = result.distance_mm;

    if (result.range_status == 0)
    {
      data_valid = true;
      return 0;
    }
  }

  data_valid = false;
  return -1;
}

bool ProximitySensor120cm::IsDataValid()
{
  return data_valid;
}

uint16_t ProximitySensor120cm::GetDistance()
{
  return distance_mm;
}

} /* namespace sensor_drivers */
} /* namespace stm32_code */
