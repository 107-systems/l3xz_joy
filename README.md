<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `l3xz_joy`
========================

PS3 joystick driver for feeding l3xz_teleop node.

#### How-to-build
```bash
colcon_ws/src$ git clone https://github.com/107-systems/l3xz_joy
colcon_ws$ source /opt/ros/galactic/setup.bash
colcon_ws$ colcon build
```

#### How-to-run
```bash
colcon_ws$ source install/setup.bash
colcon_ws$ ros2 launch l3xz_joy joy.py
```
Display the published messages by
```bash
ros2 topic echo /l3xz/joy
```
