from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("image_topic", default_value=""),
            DeclareLaunchArgument("compressed_image_topic", default_value=""),
            DeclareLaunchArgument("require_fresh_image", default_value="false"),
            DeclareLaunchArgument("force_cpu", default_value="false"),
            DeclareLaunchArgument("gpu_device", default_value="0"),
            Node(
                package="vln_nav2_bridge",
                executable="vln_node_local",
                name="vln_bridge_node_local",
                output="screen",
                parameters=[
                    {"use_sim_time": True},
                    {"image_topic": LaunchConfiguration("image_topic")},
                    {"compressed_image_topic": LaunchConfiguration("compressed_image_topic")},
                    {"require_fresh_image": LaunchConfiguration("require_fresh_image")},
                    {"force_cpu": LaunchConfiguration("force_cpu")},
                    {"gpu_device": LaunchConfiguration("gpu_device")},
                ],
            )
        ]
    )
