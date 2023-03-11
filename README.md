<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `l3xz_joy`
========================
[![Build Status](https://github.com/107-systems/l3xz_joy/actions/workflows/ros2.yml/badge.svg)](https://github.com/107-systems/l3xz_joy/actions/workflows/ros2.yml)
[![Spell Check status](https://github.com/107-systems/l3xz_joy/actions/workflows/spell-check.yml/badge.svg)](https://github.com/107-systems/l3xz_joy/actions/workflows/spell-check.yml)

A generic PS3 joystick driver for feeding the [l3xz_teleop](https://github.com/107-systems/l3xz_teleop) node.

<p align="center">
  <a href="https://github.com/107-systems/l3xz"><img src="https://raw.githubusercontent.com/107-systems/.github/main/logo/l3xz-logo-memento-mori-github.png" width="40%"></a>
</p>

#### How-to-build
```bash
cd $COLCON_WS/src
git clone https://github.com/107-systems/l3xz_joy
cd $COLCON_WS
source /opt/ros/humble/setup.bash
colcon build --packages-select l3xz_joy
```

#### How-to-run
```bash
cd $COLCON_WS
. install/setup.bash
ros2 launch l3xz_joy joy.py
```
Display the published messages by
```bash
ros2 topic echo /l3xz/joy
```

#### Interface Documentation
##### Published Topics
| Default name | Type |
|:-:|:-:|
| `/joy` | [`sensor_msgs/Joy`](http://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html) |

##### Parameters
| Name | Default | Description |
|:-:|:-:|-|
| `joy_dev_node` | `/dev/input/js0` | Name of input device node under which joystick is registered in Linux. |
| `joy_topic` | `/joy` | Name of topic for publishing the joystick message. |
| `joy_topic_publish_period_ms` | 50 | Publishing period for the joystick message in milliseconds (ms). |
| `joy_deadzone` | 0.01 | Deadzone of the joystick, if the axis value is below this value than 0.0 is reported instead. |
