
#ifndef STM32CODE_SENSORINTERFACE_PROXIMITYSENSOR10CM_H
#define STM32CODE_SENSORINTERFACE_PROXIMITYSENSOR10CM_H

/* Project .h files */
#include "../stm32h7xx_hal.h"
#include "../common_types.h"
#include "../../../Drivers/Adafruit_VL6180X/Adafruit_VL6180X.h"

namespace stm32_code
{
namespace sensor_interface
{

/*!
 * @brief Class representing driver control of one BNO055 nine axis imu.
 *
 * Class representing driver control of one BNO055 nine axis imu.
 */
class ProximitySensor62cm
{
public:
  ProximitySensor62cm();
  ~ProximitySensor62cm();
  Status Init(I2C_HandleTypeDef* hi2c);
  Scalar<uint8_t> GetDistance();
  Scalar<float> GetIlluminance();

private:
  Adafruit_VL6180X vl;
  uint8_t i2c_address = (0x29 << 1);
};

} /* namespace sensor_interface */
} /* namespace stm32_code */


#endif /* STM32CODE_SENSORINTERFACE_PROXIMITYSENSOR10CM_H */
