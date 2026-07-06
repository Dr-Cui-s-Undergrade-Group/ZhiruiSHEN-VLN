from setuptools import setup

package_name = "vln_nav2_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/pointcloud_to_scan_bridge.launch.py",
                "launch/rviz_track.launch.py",
                "launch/vln_nav2_bridge_local.launch.py",
            ],
        ),
        (
            "share/" + package_name + "/config",
            [
                "config/robot_track.rviz",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="bluepoisons",
    maintainer_email="bluepoisons@example.com",
    description="Local VLN to Nav2 bridge for Node 5/6.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vln_node_local = vln_nav2_bridge.vln_node_local:main",
            "node6_auto_trials = vln_nav2_bridge.node6_auto_trials:main",
            "node6_map_preflight = vln_nav2_bridge.node6_map_preflight:main",
            "node8_scan_accumulator = vln_nav2_bridge.node8_scan_accumulator:main",
            "node8_scan_map_residual = vln_nav2_bridge.node8_scan_map_residual:main",
            "node8_active_visual_relocalization = vln_nav2_bridge.node8_active_visual_relocalization:main",
            "static_base_footprint_tf = vln_nav2_bridge.static_base_footprint_tf:main",
            "node8_waypoint_driver = vln_nav2_bridge.node8_waypoint_driver:main",
            "node8_odom_truth_mapper = vln_nav2_bridge.node8_odom_truth_mapper:main",
            "odom_tf_broadcaster = vln_nav2_bridge.odom_tf_broadcaster:main",
            "odom_to_path = vln_nav2_bridge.odom_to_path:main",
            "pointcloud_to_map_cloud = vln_nav2_bridge.pointcloud_to_map_cloud:main",
            "save_pointcloud_snapshot = vln_nav2_bridge.save_pointcloud_snapshot:main",
        ],
    },
)
