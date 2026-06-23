from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("cloud_topic", default_value="/front_3d_lidar/lidar_points"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("target_frame", default_value="base_link"),
            DeclareLaunchArgument("transform_tolerance", default_value="0.2"),
            DeclareLaunchArgument("min_height", default_value="-0.5"),
            DeclareLaunchArgument("max_height", default_value="0.2"),
            # Keep this launch as a direct pointcloud_to_laserscan baseline.
            # Node 8's preferred path is node8_scan_accumulator, which rolls
            # multiple sparse Isaac point-cloud frames into a denser scan.
            DeclareLaunchArgument("angle_min", default_value="-3.14159"),
            DeclareLaunchArgument("angle_max", default_value="3.14159"),
            DeclareLaunchArgument("angle_increment", default_value="0.0087"),
            DeclareLaunchArgument("scan_time", default_value="0.2"),
            DeclareLaunchArgument("range_min", default_value="0.1"),
            DeclareLaunchArgument("range_max", default_value="30.0"),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {"target_frame": LaunchConfiguration("target_frame")},
                    {"transform_tolerance": LaunchConfiguration("transform_tolerance")},
                    {"min_height": LaunchConfiguration("min_height")},
                    {"max_height": LaunchConfiguration("max_height")},
                    {"angle_min": LaunchConfiguration("angle_min")},
                    {"angle_max": LaunchConfiguration("angle_max")},
                    {"angle_increment": LaunchConfiguration("angle_increment")},
                    {"scan_time": LaunchConfiguration("scan_time")},
                    {"range_min": LaunchConfiguration("range_min")},
                    {"range_max": LaunchConfiguration("range_max")},
                ],
                remappings=[
                    ("cloud_in", LaunchConfiguration("cloud_topic")),
                    ("scan", LaunchConfiguration("scan_topic")),
                ],
            ),
        ]
    )
