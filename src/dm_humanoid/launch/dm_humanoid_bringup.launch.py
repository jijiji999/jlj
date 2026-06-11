from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_joy = LaunchConfiguration("use_joy")
    use_imu = LaunchConfiguration("use_imu")
    use_system_check = LaunchConfiguration("use_system_check")
    require_imu = LaunchConfiguration("require_imu")
    require_joy = LaunchConfiguration("require_joy")
    can_bridge_managed_externally = LaunchConfiguration("can_bridge_managed_externally")
    joy_device_path = LaunchConfiguration("joy_device_path")
    joy_topic = LaunchConfiguration("joy_topic")
    imu_port = LaunchConfiguration("imu_port")
    imu_baud = LaunchConfiguration("imu_baud")
    config_path = LaunchConfiguration("config_path")
    motor_config_path = LaunchConfiguration("motor_config_path")
    startup_mode = LaunchConfiguration("startup_mode")
    control_hz = LaunchConfiguration("control_hz")
    policy_hz = LaunchConfiguration("policy_hz")

    return LaunchDescription([
        DeclareLaunchArgument("use_joy", default_value="true"),
        DeclareLaunchArgument("use_imu", default_value="true"),
        DeclareLaunchArgument("use_system_check", default_value="true"),
        DeclareLaunchArgument("require_joy", default_value="true"),
        DeclareLaunchArgument("require_imu", default_value="true"),
        DeclareLaunchArgument("can_bridge_managed_externally", default_value="true"),
        DeclareLaunchArgument(
            "joy_device_path",
            default_value="/dev/input/by-id/usb-S_TGZ_Controller_3E529690-joystick"),
        DeclareLaunchArgument("joy_topic", default_value="joy"),
        DeclareLaunchArgument("imu_port", default_value="/dev/dm_imu"),
        DeclareLaunchArgument("imu_baud", default_value="921600"),
        DeclareLaunchArgument(
            "config_path",
            default_value="/home/sliouzhou04/jlj_ws/src/dm_humanoid/config/dm_humanoid_29.yaml"),
        DeclareLaunchArgument(
            "motor_config_path",
            default_value="/home/sliouzhou04/jlj_ws/src/dm_motor/config/dm_motor_29.yaml"),
        DeclareLaunchArgument("startup_mode", default_value="passive"),
        DeclareLaunchArgument("control_hz", default_value="200.0"),
        DeclareLaunchArgument("policy_hz", default_value="50.0"),
        LogInfo(
            condition=IfCondition(can_bridge_managed_externally),
            msg="CAN bridge is expected to be started externally before bringup."),
        LogInfo(
            condition=UnlessCondition(can_bridge_managed_externally),
            msg="This launch does not yet spawn the CAN bridge automatically. Start start_can_bridge.sh first."),
        Node(
            package="dm_joy",
            executable="dm_joy_node",
            name="dm_joy_node",
            output="screen",
            condition=IfCondition(use_joy),
            parameters=[{
                "device_path": joy_device_path,
                "joy_topic": joy_topic,
            }],
        ),
        Node(
            package="dm_imu",
            executable="dm_imu_node",
            name="dm_imu_node",
            output="screen",
            condition=IfCondition(use_imu),
            parameters=[{
                "port": imu_port,
                "baud": imu_baud,
            }],
        ),
        Node(
            package="dm_humanoid",
            executable="dm_humanoid_control_node",
            name="dm_humanoid_control_node",
            output="screen",
            parameters=[{
                "config_path": config_path,
                "motor_config_path": motor_config_path,
                "joy_topic": joy_topic,
                "startup_mode": startup_mode,
                "control_hz": control_hz,
                "policy_hz": policy_hz,
            }],
        ),
        Node(
            package="dm_humanoid",
            executable="dm_humanoid_system_check_node",
            name="dm_humanoid_system_check_node",
            output="screen",
            condition=IfCondition(use_system_check),
            parameters=[{
                "joy_topic": joy_topic,
                "imu_topic": "imu/data",
                "raw_motor_topic": "raw_motor_states",
                "joint_states_topic": "joint_states",
                "mode_topic": "humanoid_control/mode",
                "require_joy": require_joy,
                "require_imu": require_imu,
                "require_raw_motor_states": "true",
                "require_joint_states": "true",
                "require_mode_topic": "true",
            }],
        ),
    ])
