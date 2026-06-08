# dm_joy

ROS 2 joystick driver package for a USB gamepad detected as:

- device name: `Microsoft X-Box 360 pad`
- stable path: `/dev/input/by-id/usb-S_TGZ_Controller_3E529690-joystick`

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

This controller appears as an Xbox-compatible gamepad. The monitor node uses the
labels printed on your controller while keeping the detected Linux button and axis indices:

- `button[0]`: `A`
- `button[1]`: `B`
- `button[2]`: `X`
- `button[3]`: `Y`
- `button[4]`: `L1`
- `button[5]`: `R1`
- `button[6]`: `select`
- `button[7]`: `start`
- `button[8]`: `ps`
- `button[9]`: `L3`
- `button[10]`: `R3`

- `axis[0]`: `left_stick_x`
- `axis[1]`: `left_stick_y_up_positive`
- `axis[2]`: `L2_trigger`
- `axis[3]`: `right_stick_x`
- `axis[4]`: `right_stick_y_up_positive`
- `axis[5]`: `R2_trigger`
- `axis[6]`: `dpad_x`
- `axis[7]`: `dpad_y_up_positive`

By default, the driver flips these three axes so the sign is more intuitive for robot control:

- `axis[1]`: pushing the left stick up becomes positive
- `axis[4]`: pushing the right stick up becomes positive
- `axis[7]`: pressing D-pad up becomes positive

## Permission note

If `/dev/input/js0` cannot be opened, make sure the current user has access to the
input device or is running in the active desktop session with `uaccess`.
