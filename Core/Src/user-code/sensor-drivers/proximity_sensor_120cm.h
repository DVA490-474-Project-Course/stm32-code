/*
 * proximity_sensor_120cm.h
 *
 *  Created on: Nov 12, 2024
 *      Author: vicore
 */


#ifndef STM32CODE_SENSORDRIVERS_PROXIMITYSENSOR120CM_H
#define STM32CODE_SENSORDRIVERS_PROXIMITYSENSOR120CM_H

#include "../stm32h7xx_hal.h"
#include "../../../Drivers/VL53L4CD_ULD_Driver/VL53L4CD_api.h"

namespace stm32_code
{
namespace sensor_drivers
{

class ProximitySensor120cm
{
public:
  ProximitySensor120cm();
  ~ProximitySensor120cm();
  int Init(I2C_HandleTypeDef* hi2c);
  int ReadData(I2C_HandleTypeDef* hi2c);
  bool IsDataValid();
  uint16_t GetDistance();

private:
  Dev_t address = 0x52;
  /* Status of measurements. If the status is equal to 0, the data are valid*/
  bool data_valid;
  /* Measured distance in millimeters */
  uint16_t distance_mm;
};

} /* namespace sensor_drivers */
} /* namespace stm32_code */


#endif /* STM32CODE_SENSORDRIVERS_PROXIMITYSENSOR120CM_H */
