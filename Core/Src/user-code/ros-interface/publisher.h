/*
 * ros_publisher.h
 *
 *  Created on: Dec 5, 2024
 *      Author: vicore
 */

#ifndef STM32CODE_ROSINTERFACE_PUBLISHER_H
#define STM32CODE_ROSINTERFACE_PUBLISHER_H

#include "string"

/* Project .h files */
#include "../common_types.h"
#include "../stm32h7xx_hal.h"

#include "rclc/rclc.h"
#include "std_msgs/msg/float32.h"

namespace stm32_code
{
namespace ros_interface
{

class Publisher
{
public:
  Publisher(const std::string name);
  Status Publish(float value);

private:
  rcl_allocator_t freeRTOS_allocator;

  /* micro-ROS app */
  rcl_publisher_t publisher;
  std_msgs__msg__Float32 msg;
  rclc_support_t support;
  rcl_allocator_t allocator;
  rcl_node_t node;
};

} /* namespace ros_interface */
} /* namespace stm32_code */

#endif /* STM32CODE_ROSINTERFACE_PUBLISHER_H */
