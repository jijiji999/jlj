# dm_joy

ROS 2 joystick driver package for a USB gamepad detected as:

- device name: `Microsoft X-Box 360 pad`
- stable path: `/dev/input/by-id/usb-S_TGZ_Controller_3E529620-joystick`

## Nodes

- `dm_joy_node`
  Publishes `sensor_msgs/msg/Joy` on `/joy`
- `dm_joy_monitor_node`
  Prints button names and active axis values for quick mapping tests

## Launch

```bash
ros2 launch dm_joy dm_joy.launch.py
```

## Typical mapping

This controller appears as an Xbox-compatible gamepad. The monitor node assumes:

- `button[0]`: `cross_or_A`
- `button[1]`: `circle_or_B`
- `button[2]`: `square_or_X`
- `button[3]`: `triangle_or_Y`
- `button[4]`: `L1_or_LB`
- `button[5]`: `R1_or_RB`
- `button[6]`: `select_or_back`
- `button[7]`: `start`
- `button[8]`: `ps_or_guide`
- `button[9]`: `L3`
- `button[10]`: `R3`

- `axis[0]`: `left_stick_x`
- `axis[1]`: `left_stick_y`
- `axis[2]`: `L2_or_LT`
- `axis[3]`: `right_stick_x`
- `axis[4]`: `right_stick_y`
- `axis[5]`: `R2_or_RT`
- `axis[6]`: `dpad_x`
- `axis[7]`: `dpad_y`

## Permission note

If `/dev/input/js0` cannot be opened, make sure the current user has access to the
input device or is running in the active desktop session with `uaccess`.
