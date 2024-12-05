/* publisher.h
 *==============================================================================
 * Author: Emil Åberg
 * Creation date: 2024-12-05
 * Last modified: 2024-12-05 by Emil Åberg
 * Description: Class representing a ROS node and publisher that can transmit
 * data to Raspberry Pi connected to peripheral UART5.
 * License: See LICENSE file for license details.
 *==============================================================================
 */

#ifndef STM32CODE_ROSINTERFACE_PUBLISHER_H
#define STM32CODE_ROSINTERFACE_PUBLISHER_H

/* C++ standard library headers */
#include "string"

/* Project .h files */
#include "../common_types.h"
#include "../stm32h7xx_hal.h"

/* Other .h files */
#include "rclc/rclc.h"
#include "std_msgs/msg/float32.h"

namespace stm32_code
{
namespace ros_interface
{

class Publisher
{
public:
  /*! 
   * @brief Creates a ROS node and publisher.
   * 
   * Creates a ROS node and publisher. The published value will be of message type
   * Float32.
   *
   * @param[in] name First part of the name of the node and publisher. The created
   * node and publisher will be named '<name>_node' and '<name>_publisher'
   * respectively.
   */
  Publisher(const std::string name);

  /*! 
   * @brief Publish a value.
   * 
   * Publishes a new value. The published value will be of message type
   * Float32.
   *
   * @param[in] value Value to publish.
   * 
   * @return Status indicating wheter value was successfully published.
   */
  Status Publish(float value);

private:
  /*! 
   * @brief Object to handle freertos memory allocation. Datatype defined by
   * microROS.
   */
  rcl_allocator_t freertos_allocator;

  /*! 
   * @brief Object representing the microros publisher. Datatype defined by
   * microROS.
   */
  rcl_publisher_t publisher;

  /*! 
   * @brief Object representing the ROS message. Datatype defined by
   * microROS.
   */
  std_msgs__msg__Float32 msg;

  /*! 
   * @brief Object representing the node options. Datatype defined by
   * microROS.
   */
  rclc_support_t support;

  /*! 
   * @brief Object to handle node memory allocation. Datatype defined by
   * microROS.
   */
  rcl_allocator_t allocator;

  /*! 
   * @brief Object representing the node. Datatype defined by microROS.
   */
  rcl_node_t node;
};

} /* namespace ros_interface */
} /* namespace stm32_code */

#endif /* STM32CODE_ROSINTERFACE_PUBLISHER_H */
