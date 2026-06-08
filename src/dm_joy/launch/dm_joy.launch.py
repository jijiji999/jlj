from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device_arg = DeclareLaunchArgument(
        "device_path",
        default_value="/dev/input/by-id/usb-S_TGZ_Controller_3E529690-joystick",
        description="Stable joystick device path",
    )
    joy_topic_arg = DeclareLaunchArgument(
        "joy_topic",
        default_value="joy",
        description="Published sensor_msgs/msg/Joy topic",
    )
    use_monitor_arg = DeclareLaunchArgument(
        "use_monitor",
        default_value="true",
        description="Whether to start the monitor node",
    )
    invert_left_stick_y_arg = DeclareLaunchArgument(
        "invert_left_stick_y",
        default_value="true",
        description="Make pushing the left stick up publish positive Y",
    )
    invert_right_stick_y_arg = DeclareLaunchArgument(
        "invert_right_stick_y",
        default_value="true",
        description="Make pushing the right stick up publish positive Y",
    )
    invert_dpad_y_arg = DeclareLaunchArgument(
        "invert_dpad_y",
        default_value="true",
        description="Make pressing D-pad up publish positive Y",
    )

    joy_node = Node(
        package="dm_joy",
        executable="dm_joy_node",
        name="dm_joy_node",
        output="screen",
        parameters=[
            {
                "device_path": LaunchConfiguration("device_path"),
                "joy_topic": LaunchConfiguration("joy_topic"),
                "invert_left_stick_y": LaunchConfiguration("invert_left_stick_y"),
                "invert_right_stick_y": LaunchConfiguration("invert_right_stick_y"),
                "invert_dpad_y": LaunchConfiguration("invert_dpad_y"),
            }
        ],
    )

    joy_monitor_node = Node(
        package="dm_joy",
        executable="dm_joy_monitor_node",
        name="dm_joy_monitor_node",
        output="screen",
        parameters=[
            {
                "joy_topic": LaunchConfiguration("joy_topic"),
            }
        ],
        condition=IfCondition(LaunchConfiguration("use_monitor")),
    )

    return LaunchDescription(
        [
            device_arg,
            joy_topic_arg,
            use_monitor_arg,
            invert_left_stick_y_arg,
            invert_right_stick_y_arg,
            invert_dpad_y_arg,
            joy_node,
            joy_monitor_node,
        ]
    )
