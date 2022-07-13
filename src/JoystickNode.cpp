/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_joy/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz_joy/JoystickNode.h>

#include <chrono>
#include <limits>
#include <numeric>

#include <l3xz_joy/PS3_Const.h>

/**************************************************************************************
 * CTOR/DTOR 
 **************************************************************************************/

JoystickNode::JoystickNode()
: Node("l3xz_joy")
,_joy_msg{
    []()
    {
      sensor_msgs::msg::Joy msg;

      msg.header.frame_id = "joy";
      msg.axes.resize(NUM_AXES);
      msg.buttons.resize(NUM_BUTTONS);

      return msg;
    } ()
  }
, _joy_mtx{}
, _joy_thread{}
, _joy_thread_active{false}
{
  declare_parameter("joy_dev_node", "/dev/input/js0");
  declare_parameter("joy_topic", "joy");

  _joy_pub = create_publisher<sensor_msgs::msg::Joy>
    (get_parameter("joy_topic").as_string(), 10);
  
  _joy_pub_timer = create_wall_timer
    (std::chrono::milliseconds(50), [this]() { this->joystickPubFunc(); });

  _joystick   = std::make_shared<Joystick>(get_parameter("joy_dev_node").as_string());
  _joy_thread = std::thread([this]() { this->joystickThreadFunc(); });
}

JoystickNode::~JoystickNode()
{
  _joy_thread_active = false;
  _joy_thread.join();
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void JoystickNode::joystickThreadFunc()
{
  _joy_thread_active = true;

  while (_joy_thread_active)
  {
    JoystickEvent const evt = _joystick->update();

    if (evt.isInit())
      continue;

    if (evt.isAxis())
    {
      RCLCPP_INFO(get_logger(), "Axis %d: %d", evt.number, evt.value);

      float const axis_scaled_val = static_cast<float>(evt.value) / static_cast<float>(std::numeric_limits<int16_t>::max());

      std::lock_guard<std::mutex> lock(_joy_mtx);
      if (isValidAxisId(evt.number))
        _joy_msg.axes[AXIS_TO_ARRAY_MAP.at(evt.number)] = axis_scaled_val;
    }

    if (evt.isButton()) {
      RCLCPP_INFO(get_logger(), "Button %d: %d", evt.number, evt.value);
      
      std::lock_guard<std::mutex> lock(_joy_mtx);
      if (isValidButtonId(evt.number))
        _joy_msg.buttons[BUTTON_TO_ARRAY_MAP.at(evt.number)] = evt.value;
    }
  }
}

void JoystickNode::joystickPubFunc()
{
  std::lock_guard<std::mutex> lock(_joy_mtx);

  _joy_msg.header.stamp = this->now();

  _joy_pub->publish(_joy_msg);
}
