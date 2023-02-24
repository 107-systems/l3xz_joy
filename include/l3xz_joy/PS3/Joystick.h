/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_joy/graphs/contributors.
 */

#ifndef JOYSTICK_H_
#define JOYSTICK_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <string>
#include <optional>

#include "JoystickEvent.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::joystick::ps3
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Joystick
{
public:

  Joystick(std::string const &dev_node);

  ~Joystick();

  std::optional<JoystickEvent> update();

private:
  int _fd;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::joystick::ps3 */

#endif /* JOYSTICK_H_ */
