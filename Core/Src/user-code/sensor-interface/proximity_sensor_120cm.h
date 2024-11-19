/*
 * proximity_sensor_120cm.h
 *
 *  Created on: Nov 12, 2024
 *      Author: vicore
 */


#ifndef STM32CODE_SENSORINTERFACE_PROXIMITYSENSOR120CM_H
#define STM32CODE_SENSORINTERFACE_PROXIMITYSENSOR120CM_H

#include "../stm32h7xx_hal.h"
#include "../common_types.h"
#include "../../../Drivers/VL53L4CD_ULD_Driver/VL53L4CD_api.h"

namespace stm32_code
{
namespace sensor_interface
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

} /* namespace sensor_interface */
} /* namespace stm32_code */


#endif /* STM32CODE_SENSORINTERFACE_PROXIMITYSENSOR120CM_H */
