# DM IMU udev Setup

This IMU can be bound to a stable device alias `/dev/dm_imu` so the ROS 2 driver
does not depend on changing `/dev/ttyACM*` numbers.

## Detected device identifiers

- `idVendor`: `6877`
- `idProduct`: `4d55`
- `serial`: `DMIMU20250212`
- current USB product string: `DM-IMU-L1`

## Install rule

Copy the rule into the system udev directory:

```bash
sudo cp src/dm_imu/config/99-dm-imu.rules /etc/udev/rules.d/99-dm-imu.rules
```

Reload rules and trigger the device:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Then unplug and replug the IMU, or verify directly with:

```bash
ls -l /dev/dm_imu
```

## Driver usage

The package defaults now point to:

```bash
/dev/dm_imu
```

You can still override it from launch if needed:

```bash
ros2 launch dm_imu imu_visual_test.launch.py port:=/dev/ttyACM0
```

## Permission note

The rule assigns:

- group: `dialout`
- mode: `0660`

Your user should be in the `dialout` group for normal access:

```bash
sudo usermod -aG dialout $USER
```

After that, log out and log back in.
