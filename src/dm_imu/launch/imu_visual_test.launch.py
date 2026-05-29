from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory("dm_imu")
    rviz_config = os.path.join(package_share, "rviz", "imu_visual_test.rviz")

    port_arg = DeclareLaunchArgument(
        "port",
        default_value="/dev/dm_imu",
        description="Serial port used by the IMU driver",
    )
    baud_arg = DeclareLaunchArgument(
        "baud",
        default_value="921600",
        description="IMU serial baud rate",
    )
    start_driver_arg = DeclareLaunchArgument(
        "start_driver",
        default_value="true",
        description="Whether to start dm_imu_node",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Whether to start RViz",
    )
    use_rqt_plot_arg = DeclareLaunchArgument(
        "use_rqt_plot",
        default_value="false",
        description="Whether to start rqt_plot for realtime curves",
    )

    imu_driver = Node(
        package="dm_imu",
        executable="dm_imu_node",
        name="dm_imu_node",
        output="screen",
        parameters=[
            {
                "port": LaunchConfiguration("port"),
                "baud": LaunchConfiguration("baud"),
            }
        ],
        condition=IfCondition(LaunchConfiguration("start_driver")),
    )

    imu_visual_test = Node(
        package="dm_imu",
        executable="imu_visual_test_node",
        name="imu_visual_test_node",
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    rqt_plot = Node(
        package="rqt_plot",
        executable="rqt_plot",
        name="imu_rqt_plot",
        output="screen",
        arguments=[
            "/imu_visual_test/euler/vector/x:/vector/y:/vector/z",
            "/imu_visual_test/angular_velocity/vector/x:/vector/y:/vector/z",
            "/imu_visual_test/linear_acceleration/vector/x:/vector/y:/vector/z",
        ],
        condition=IfCondition(LaunchConfiguration("use_rqt_plot")),
    )

    return LaunchDescription(
        [
            port_arg,
            baud_arg,
            start_driver_arg,
            use_rviz_arg,
            use_rqt_plot_arg,
            imu_driver,
            imu_visual_test,
            rviz,
            rqt_plot,
        ]
    )
