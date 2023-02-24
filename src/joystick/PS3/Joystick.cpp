/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_joy/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz_joy/PS3/Joystick.h>

#include <poll.h>
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

std::optional<JoystickEvent> Joystick::update()
{
  struct pollfd pfd;
  pfd.fd = _fd;
  pfd.events = POLLIN;

  int const rc = poll(&pfd, 1 /* 1 event */, 100 /* ms */);

  if      (rc < 0) /* Error. */
    throw std::runtime_error("Joystick::update: error on 'poll': " + std::string(strerror(errno)));
  else if (rc == 0) /* Timeout. */
    return std::nullopt;
  else {
    JoystickEvent evt;
    read(_fd, &evt, sizeof(JoystickEvent));
    return evt;
  }
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::joystick::ps3 */
