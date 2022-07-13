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
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>

#include "Joystick.h"

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class JoystickNode : public rclcpp::Node
{
public:
   JoystickNode();
  ~JoystickNode();

private:
  rclcpp::TimerBase::SharedPtr _joy_pub_timer;
 
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr _joy_pub;
  sensor_msgs::msg::Joy _joy_msg;

  std::shared_ptr<Joystick> _joystick;
  std::mutex _joy_mtx;
  std::thread _joy_thread;
  std::atomic<bool> _joy_thread_active;

  void joystickThreadFunc();
  void joystickPubFunc();
};
