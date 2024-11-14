/*
 * proximity_sensor_120cm.h
 *
 *  Created on: Nov 12, 2024
 *      Author: vicore
 */


#ifndef STM32CODE_SENSORDRIVERS_PROXIMITYSENSOR120CM_H
#define STM32CODE_SENSORDRIVERS_PROXIMITYSENSOR120CM_H

#include "../stm32h7xx_hal.h"
#include "../common_types.h"
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
  Status Init(I2C_HandleTypeDef* hi2c);
  Scalar<uint16_t> GetDistance();

private:
  Dev_t address = 0x52;
  I2C_HandleTypeDef* hi2c;
};

} /* namespace sensor_drivers */
} /* namespace stm32_code */


#endif /* STM32CODE_SENSORDRIVERS_PROXIMITYSENSOR120CM_H */
