/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_joy/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz_joy/PS3/Joystick.h>

#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#include <stdexcept>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::joystick::ps3
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Joystick::Joystick(std::string const & dev_node)
: _fd{open(dev_node.c_str(), O_RDONLY)}
{
  if (_fd < 0)
    throw std::runtime_error("Joystick::Joystick: error on 'fopen': " + std::string(strerror(errno)));
}

Joystick::~Joystick()
{
  close(_fd);
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

JoystickEvent Joystick::update()
{
	JoystickEvent evt;
  
  if (read(_fd, &evt, sizeof(JoystickEvent)) != sizeof(JoystickEvent))
    throw std::runtime_error("Joystick::update: error on 'fread': " + std::string(strerror(errno)));

  return evt;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::joystick::ps3 */
