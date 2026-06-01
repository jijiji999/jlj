from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution(
        [FindPackageShare("dm_test"), "config", "dm_test_trajectories.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("config_path", default_value=default_config),
            DeclareLaunchArgument(
                "trajectory_name", default_value="left_arm_small_range"
            ),
            DeclareLaunchArgument("trajectory_csv_path", default_value=""),
            DeclareLaunchArgument("time_column_name", default_value="time"),
            DeclareLaunchArgument("command_rate_hz", default_value="100.0"),
            DeclareLaunchArgument("playback_speed", default_value="1.0"),
            DeclareLaunchArgument("csv_row_rate_hz", default_value="0.0"),
            DeclareLaunchArgument("command_timeout_ms", default_value="20"),
            DeclareLaunchArgument("start_delay_sec", default_value="1.0"),
            DeclareLaunchArgument("auto_enable_on_start", default_value="true"),
            DeclareLaunchArgument("auto_disable_on_shutdown", default_value="false"),
            Node(
                package="dm_test",
                executable="dm_trajectory_test_node",
                name="dm_trajectory_test_node",
                output="screen",
                parameters=[
                    {
                        "config_path": LaunchConfiguration("config_path"),
                        "trajectory_name": LaunchConfiguration("trajectory_name"),
                        "trajectory_csv_path": LaunchConfiguration("trajectory_csv_path"),
                        "time_column_name": LaunchConfiguration("time_column_name"),
                        "command_rate_hz": LaunchConfiguration("command_rate_hz"),
                        "playback_speed": LaunchConfiguration("playback_speed"),
                        "csv_row_rate_hz": LaunchConfiguration("csv_row_rate_hz"),
                        "command_timeout_ms": LaunchConfiguration("command_timeout_ms"),
                        "start_delay_sec": LaunchConfiguration("start_delay_sec"),
                        "auto_enable_on_start": LaunchConfiguration(
                            "auto_enable_on_start"
                        ),
                        "auto_disable_on_shutdown": LaunchConfiguration(
                            "auto_disable_on_shutdown"
                        ),
                    }
                ],
            ),
        ]
    )
