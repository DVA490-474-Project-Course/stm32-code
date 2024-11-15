#include "../stm32h7xx_hal.h"
#include "../common_types.h"

namespace stm32_code
{
namespace sensor_drivers
{

Status ReadByte(I2C_HandleTypeDef* hi2c, uint8_t i2c_address, uint8_t register_address, uint8_t* value)
{
  uint8_t data_write[1];
  uint8_t data_read[1];
  uint8_t status = 0;

  data_write[0] = register_address;
  status = HAL_I2C_Master_Transmit(hi2c, i2c_address, data_write, 1, 100);
  status = HAL_I2C_Master_Receive(hi2c, i2c_address, data_read, 1, 100);
  *value = data_read[0];

  if (status == 0)
  {
    return Status::kOk;
  }
  else
  {
    return Status::kNotOk;
  }
}

Status ReadBytes(I2C_HandleTypeDef* hi2c, uint8_t i2c_address, uint8_t register_address, uint8_t buffer[], int buffer_size)
{
  uint8_t data_write[1];
  uint8_t status = 0;

  data_write[0] = register_address;
  status = HAL_I2C_Master_Transmit(hi2c, i2c_address, data_write, 1, 100);
  status = HAL_I2C_Master_Receive(hi2c, i2c_address, buffer, buffer_size, 100);

  if (status == 0)
  {
    return Status::kOk;
  }
  else
  {
    return Status::kNotOk;
  }
}

Status WriteByte(I2C_HandleTypeDef* hi2c, uint8_t i2c_address, uint8_t register_address, uint8_t value)
{
  uint8_t data_write[2];
  uint8_t status = 0;

  data_write[0] = register_address;
  data_write[1] = value;
  status = HAL_I2C_Master_Transmit(hi2c, i2c_address, data_write, 2, 100);

  if (status == 0)
  {
    return Status::kOk;
  }
  else
  {
    return Status::kNotOk;
  }
}

} /* namespace sensor_drivers */
} /* namespace stm32_code */
