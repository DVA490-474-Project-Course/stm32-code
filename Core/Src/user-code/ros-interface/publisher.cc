/* publisher.cc
 *==============================================================================
 * Author: Emil Åberg
 * Creation date: 2024-12-05
 * Last modified: 2024-12-05 by Emil Åberg
 * Description: Class representing a ROS node and publisher that can transmit
 * data to Raspberry Pi connected to peripheral UART5.
 * License: See LICENSE file for license details.
 *==============================================================================
 */

/* C++ standard library headers */
#include "string"

/* Related .h file */
#include "../ros-interface/publisher.h"

/* Project .h files */
#include "../common_types.h"
#include "../stm32h7xx_hal.h"

/* Other .h files */
#include "rcl/rcl.h"
#include "rcl/error_handling.h"
#include "rclc/rclc.h"
#include "rmw_microros/rmw_microros.h"
#include "std_msgs/msg/float32.h"

/* UART5 is connected to Raspberry Pi */
extern UART_HandleTypeDef huart5;

extern "C"
{

/* Declarations for functions used by microROS to send topic data via UART */
bool cubemx_transport_open(struct uxrCustomTransport* transport);
bool cubemx_transport_close(struct uxrCustomTransport* transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport,
    const uint8_t* buf, size_t len, uint8_t* err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport,
    uint8_t* buf, size_t len, int timeout, uint8_t* err);
void* microros_allocate(size_t size, void* state);
void microros_deallocate(void* pointer, void* state);
void* microros_reallocate(void* pointer, size_t size, void* state);
void* microros_zero_allocate(size_t number_of_elements, size_t size_of_element,
    void* state);

}

namespace stm32_code
{
namespace ros_interface
{

/* Creates a ROS node and publisher instance */
Publisher::Publisher(const std::string topic_name)
{
  /* micro-ROS configuration */
  rmw_uros_set_custom_transport(
	true,
	(void *) &huart5,
	cubemx_transport_open,
	cubemx_transport_close,
	cubemx_transport_write,
	cubemx_transport_read);
  freertos_allocator = rcutils_get_zero_initialized_allocator();
  freertos_allocator.allocate = microros_allocate;
  freertos_allocator.deallocate = microros_deallocate;
  freertos_allocator.reallocate = microros_reallocate;
  freertos_allocator.zero_allocate =  microros_zero_allocate;
  allocator = rcl_get_default_allocator();

  /* create init_options */
  rclc_support_init(&support, 0, NULL, &allocator);

  /* create node */
  rclc_node_init_default(&node, (topic_name + "_node").c_str(), "", &support);

  /* create publisher */
  rclc_publisher_init_default(
	&publisher,
	&node,
	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
	topic_name.c_str());
}

/* Publishes a new float value */
Status Publisher::Publish(float value)
{
  rcl_ret_t ret;

  /* Publish value */
  msg.data = value;
  ret = rcl_publish(&publisher, &msg, NULL);

  /* Return status indicator */
  if (ret == RCL_RET_OK)
  {
    return Status::kOk;
  }
  else
  {
    return Status::kNotOk;
  }
}

} /* namespace ros_interface */
} /* namespace stm32_code */
