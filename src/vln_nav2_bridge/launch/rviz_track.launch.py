from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("vln_nav2_bridge"), "config", "robot_track.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("rviz_config", default_value=rviz_config),
            DeclareLaunchArgument("odom_topic", default_value="/chassis/odom"),
            DeclareLaunchArgument("path_topic", default_value="/executed_path"),
            DeclareLaunchArgument("cloud_topic", default_value="/front_3d_lidar/lidar_points"),
            DeclareLaunchArgument("map_cloud_topic", default_value="/front_3d_lidar/map_points"),
            DeclareLaunchArgument("target_frame", default_value="map"),
            DeclareLaunchArgument("start_odom_tf", default_value="true"),
            DeclareLaunchArgument("start_base_footprint_tf", default_value="false"),
            DeclareLaunchArgument("start_path", default_value="true"),
            DeclareLaunchArgument("start_map_cloud", default_value="true"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            Node(
                package="vln_nav2_bridge",
                executable="odom_tf_broadcaster",
                name="odom_tf_broadcaster",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_odom_tf")),
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    {"odom_topic": LaunchConfiguration("odom_topic")},
                ],
            ),
            Node(
                package="vln_nav2_bridge",
                executable="static_base_footprint_tf",
                name="static_base_footprint_tf",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_base_footprint_tf")),
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
            Node(
                package="vln_nav2_bridge",
                executable="odom_to_path",
                name="odom_to_path",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_path")),
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    {"odom_topic": LaunchConfiguration("odom_topic")},
                    {"path_topic": LaunchConfiguration("path_topic")},
                    {"target_frame": LaunchConfiguration("target_frame")},
                ],
            ),
            Node(
                package="vln_nav2_bridge",
                executable="pointcloud_to_map_cloud",
                name="pointcloud_to_map_cloud",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_map_cloud")),
                parameters=[
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                    {"cloud_topic": LaunchConfiguration("cloud_topic")},
                    {"output_topic": LaunchConfiguration("map_cloud_topic")},
                    {"target_frame": LaunchConfiguration("target_frame")},
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_rviz")),
                arguments=["-d", LaunchConfiguration("rviz_config")],
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            ),
        ]
    )
