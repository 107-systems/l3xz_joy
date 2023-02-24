/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_joy/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz_joy/Node.h>

#include <chrono>
#include <limits>
#include <numeric>

#include <l3xz_joy/PS3/PS3_Const.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::joystick
{

/**************************************************************************************
 * CTOR/DTOR 
 **************************************************************************************/

Node::Node()
: rclcpp::Node("l3xz_joy")
,_joy_msg{
    []()
    {
      sensor_msgs::msg::Joy msg;

      msg.header.frame_id = "joy";
      msg.axes.resize(ps3::NUM_AXES);
      msg.buttons.resize(ps3::NUM_BUTTONS);

      return msg;
    } ()
  }
, _joy_mtx{}
, _joy_thread{}
, _joy_thread_active{false}
{
  declare_parameter("joy_dev_node", "/dev/input/js0");
  declare_parameter("joy_topic", "joy");
  declare_parameter("joy_topic_publish_period_ms", 50);
  declare_parameter("joy_deadzone", 0.01);

  _joy_pub = create_publisher<sensor_msgs::msg::Joy>
    (get_parameter("joy_topic").as_string(), 10);
  
  _joy_pub_timer = create_wall_timer
    (std::chrono::milliseconds(get_parameter("joy_topic_publish_period_ms").as_int()), [this]() { this->joystickPubFunc(); });

  _joystick   = std::make_shared<ps3::Joystick>(get_parameter("joy_dev_node").as_string());
  _joy_thread = std::thread([this]() { this->joystickThreadFunc(); });
}

Node::~Node()
{
  _joy_thread_active = false;
  _joy_thread.join();
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void Node::joystickThreadFunc()
{
  _joy_thread_active = true;

  RCLCPP_WARN(get_logger(), "Please press [START] on your PS3 joystick.");

  while (_joy_thread_active)
  {
    std::optional<ps3::JoystickEvent> const evt = _joystick->update();

    /* Skip the event processing if we have not received a valid
     * joystick event. This is possible if a timeout or a read
     * error have occurred.
     */
    if (!evt.has_value())
      continue;

    if (evt.value().isInit()) {
      RCLCPP_INFO_ONCE(get_logger(), "PS3 joystick has been successfully initialized.");
      continue;
    }

    if (evt.value().isAxis())
    {
      RCLCPP_DEBUG(get_logger(), "Axis %d: %d", evt.value().number, evt.value().value);

      if (ps3::isValidAxisId(evt.value().number))
      {
        std::lock_guard<std::mutex> lock(_joy_mtx);

        float const axis_scaled_val = static_cast<float>(evt.value().value) / static_cast<float>(std::numeric_limits<int16_t>::max());

        if (abs(axis_scaled_val) > get_parameter("joy_deadzone").as_double())
          _joy_msg.axes[ps3::AXIS_TO_ARRAY_MAP.at(evt.value().number)] = axis_scaled_val;
        else
          _joy_msg.axes[ps3::AXIS_TO_ARRAY_MAP.at(evt.value().number)] = 0.0;
      }
    }

    if (evt.value().isButton()) {
      RCLCPP_DEBUG(get_logger(), "Button %d: %d", evt.value().number, evt.value().value);
      
      std::lock_guard<std::mutex> lock(_joy_mtx);
      if (ps3::isValidButtonId(evt.value().number))
        _joy_msg.buttons[ps3::BUTTON_TO_ARRAY_MAP.at(evt.value().number)] = evt.value().value;
    }
  }
}

void Node::joystickPubFunc()
{
  std::lock_guard<std::mutex> lock(_joy_mtx);

  _joy_msg.header.stamp = this->now();

  _joy_pub->publish(_joy_msg);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::joystick */
