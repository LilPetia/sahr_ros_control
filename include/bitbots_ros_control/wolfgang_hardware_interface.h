#ifndef BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_WOLFGANG_HARDWARE_INTERFACE_H_
#define BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_WOLFGANG_HARDWARE_INTERFACE_H_

#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <bitbots_ros_control/utils.h>
#include <bitbots_ros_control/dynamixel_servo_hardware_interface.h>
#include <bitbots_ros_control/hardware_interface.h>
#include <rcl_interfaces/msg/list_parameters_result.hpp>

namespace bitbots_ros_control {

class WolfgangHardwareInterface {
 public:
  WolfgangHardwareInterface(rclcpp::Node::SharedPtr nh);

  bool init();

  void read(const rclcpp::Time &t, const rclcpp::Duration &dt);

  void write(const rclcpp::Time &t, const rclcpp::Duration &dt);
  DynamixelServoHardwareInterface servo_interface_;

 private:
  bool create_interfaces(std::vector<std::pair<std::string, int>> dxl_devices);
  rclcpp::Node::SharedPtr nh_;

  // two dimensional list of all hardware interfaces, sorted by port
  std::vector<std::vector<bitbots_ros_control::HardwareInterface *>> interfaces_;

  // prevent unnecessary error when power is turned on
  bool first_ping_error_;
};
}

#endif //BITBOTS_ROS_CONTROL_INCLUDE_BITBOTS_ROS_CONTROL_WOLFGANG_HARDWARE_INTERFACE_H_