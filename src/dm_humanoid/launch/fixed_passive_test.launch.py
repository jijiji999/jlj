from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_path = LaunchConfiguration("config_path")
    motor_config_path = LaunchConfiguration("motor_config_path")
    startup_mode = LaunchConfiguration("startup_mode")
    control_hz = LaunchConfiguration("control_hz")
    command_timeout_ms = LaunchConfiguration("command_timeout_ms")
    fixed_passive_test_only = LaunchConfiguration("fixed_passive_test_only")
    start_joy = LaunchConfiguration("start_joy")
    joy_device_path = LaunchConfiguration("joy_device_path")
    joy_topic = LaunchConfiguration("joy_topic")
    use_joy_monitor = LaunchConfiguration("use_joy_monitor")
    invert_left_stick_y = LaunchConfiguration("invert_left_stick_y")
    invert_right_stick_y = LaunchConfiguration("invert_right_stick_y")
    invert_dpad_y = LaunchConfiguration("invert_dpad_y")

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_path",
            default_value="/home/sliouzhou04/jlj_ws/src/dm_humanoid/config/dm_humanoid_29.yaml",
            description="Path to dm_humanoid controller config",
        ),
        DeclareLaunchArgument(
            "motor_config_path",
            default_value="/home/sliouzhou04/jlj_ws/src/dm_motor/config/dm_motor_29.yaml",
            description="Path to dm_motor motor config",
        ),
        DeclareLaunchArgument(
            "startup_mode",
            default_value="passive",
            description="Startup mode for the humanoid controller",
        ),
        DeclareLaunchArgument(
            "control_hz",
            default_value="100.0",
            description="Control loop frequency",
        ),
        DeclareLaunchArgument(
            "command_timeout_ms",
            default_value="5",
            description="MIT command timeout in milliseconds",
        ),
        DeclareLaunchArgument(
            "fixed_passive_test_only",
            default_value="true",
            description="Restrict the controller to fixed/passive mode requests only",
        ),
        DeclareLaunchArgument(
            "start_joy",
            default_value="true",
            description="Whether to also start dm_joy_node",
        ),
        DeclareLaunchArgument(
            "joy_device_path",
            default_value="/dev/input/by-id/usb-S_TGZ_Controller_3E529690-joystick",
            description="Joystick device path passed to dm_joy_node",
        ),
        DeclareLaunchArgument(
            "joy_topic",
            default_value="joy",
            description="Joy topic shared by dm_joy and dm_humanoid",
        ),
        DeclareLaunchArgument(
            "use_joy_monitor",
            default_value="true",
            description="Whether to also start dm_joy_monitor_node",
        ),
        DeclareLaunchArgument(
            "invert_left_stick_y",
            default_value="true",
            description="Make pushing the left stick up publish positive Y",
        ),
        DeclareLaunchArgument(
            "invert_right_stick_y",
            default_value="true",
            description="Make pushing the right stick up publish positive Y",
        ),
        DeclareLaunchArgument(
            "invert_dpad_y",
            default_value="true",
            description="Make pressing D-pad up publish positive Y",
        ),
        Node(
            package="dm_joy",
            executable="dm_joy_node",
            name="dm_joy_node",
            output="screen",
            condition=IfCondition(start_joy),
            parameters=[
                {
                    "device_path": joy_device_path,
                    "joy_topic": joy_topic,
                    "invert_left_stick_y": invert_left_stick_y,
                    "invert_right_stick_y": invert_right_stick_y,
                    "invert_dpad_y": invert_dpad_y,
                }
            ],
        ),
        Node(
            package="dm_joy",
            executable="dm_joy_monitor_node",
            name="dm_joy_monitor_node",
            output="screen",
            condition=IfCondition(use_joy_monitor),
            parameters=[
                {
                    "joy_topic": joy_topic,
                }
            ],
        ),
        Node(
            package="dm_humanoid",
            executable="dm_humanoid_control_node",
            name="dm_humanoid_control_node",
            output="screen",
            parameters=[
                {
                    "config_path": config_path,
                    "motor_config_path": motor_config_path,
                    "joy_topic": joy_topic,
                    "startup_mode": startup_mode,
                    "control_hz": control_hz,
                    "command_timeout_ms": command_timeout_ms,
                    "fixed_passive_test_only": fixed_passive_test_only,
                }
            ],
        ),
    ])
