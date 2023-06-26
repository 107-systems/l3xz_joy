/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_joy/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <map>
#include <mutex>
#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>

#include <ros2_heartbeat/Publisher.h>

#include "PS3/Joystick.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::joystick
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Node : public rclcpp::Node
{
public:
   Node();
  ~Node();

private:
  static std::chrono::milliseconds constexpr HEARTBEAT_LOOP_RATE{100};
  heartbeat::Publisher::SharedPtr _heartbeat_pub;
  void init_heartbeat();

  rclcpp::TimerBase::SharedPtr _joy_pub_timer;

  rclcpp::QoS _joy_qos_profile;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr _joy_pub;
  sensor_msgs::msg::Joy _joy_msg;
  void init_pub();

  std::shared_ptr<ps3::Joystick> _joystick;
  std::mutex _joy_mtx;
  std::thread _joy_thread;
  std::atomic<bool> _joy_thread_active;

  void joystickThreadFunc();
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::joystick */
