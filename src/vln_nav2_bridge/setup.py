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
                "launch/vln_nav2_bridge_local.launch.py",
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
            "node8_active_visual_relocalization = vln_nav2_bridge.node8_active_visual_relocalization:main",
        ],
    },
)
