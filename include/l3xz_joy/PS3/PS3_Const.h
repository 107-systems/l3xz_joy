/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_joy/graphs/contributors.
 */

#ifndef PS3_CONST_H_
#define PS3_CONST_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <cstdint>
#include <cstdlib>

#include <map>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::joystick::ps3
{

/**************************************************************************************
 * TYPEDEF
 **************************************************************************************/

enum PS3_AxisId : uint8_t
{
  LEFT_STICK_HORIZONTAL   = 0,
  LEFT_STICK_VERTICAL     = 1,
  LEFT_REAR_2             = 2,
  RIGHT_STICK_HORIZONTAL  = 3,
  RIGHT_STICK_VERTICAL    = 4,
  RIGHT_REAR_2            = 5,
};

enum PS3_ButtonId : uint8_t
{
  PAD_UP    = 13,
  PAD_DOWN  = 14,
  PAD_RIGHT = 15,
  PAD_LEFT  = 16,
};

/**************************************************************************************
 * CONSTANT
 **************************************************************************************/

static size_t constexpr NUM_AXES    = 6;
static size_t constexpr NUM_BUTTONS = 4;

static std::map<uint8_t, size_t> AXIS_TO_ARRAY_MAP =
{
  {static_cast<uint8_t>(PS3_AxisId::LEFT_STICK_HORIZONTAL),  0},
  {static_cast<uint8_t>(PS3_AxisId::LEFT_STICK_VERTICAL),    1},
  {static_cast<uint8_t>(PS3_AxisId::LEFT_REAR_2),            2},
  {static_cast<uint8_t>(PS3_AxisId::RIGHT_STICK_HORIZONTAL), 3},
  {static_cast<uint8_t>(PS3_AxisId::RIGHT_STICK_VERTICAL),   4},
  {static_cast<uint8_t>(PS3_AxisId::RIGHT_REAR_2),           5},
};

static std::map<uint8_t, size_t> BUTTON_TO_ARRAY_MAP =
{
  {static_cast<uint8_t>(PS3_ButtonId::PAD_UP),    0},
  {static_cast<uint8_t>(PS3_ButtonId::PAD_DOWN),  1},
  {static_cast<uint8_t>(PS3_ButtonId::PAD_RIGHT), 2},
  {static_cast<uint8_t>(PS3_ButtonId::PAD_LEFT),  3},
};

/**************************************************************************************
 * FUNCTIONS
 **************************************************************************************/

inline bool isValidAxisId(uint8_t const id)
{
  if ((id == LEFT_STICK_HORIZONTAL)  ||
      (id == LEFT_STICK_VERTICAL)    ||
      (id == LEFT_REAR_2)            ||
      (id == RIGHT_STICK_HORIZONTAL) ||
      (id == RIGHT_STICK_VERTICAL)   ||
      (id == RIGHT_REAR_2))
    return true;
  else
    return false;
}

inline bool isValidButtonId(uint8_t const id)
{
  if ((id == PAD_UP)    ||
      (id == PAD_DOWN)  ||
      (id == PAD_RIGHT) ||
      (id == PAD_LEFT))
    return true;
  else
    return false;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::joystick::ps3 */

#endif /* PS3_CONST_H_ */
