from setuptools import find_packages, setup

package_name = "drone_vision_pose_publisher"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Hannes Mueller",
    maintainer_email="hannesmueller99@t-online.de",
    description="Direct vision pose publisher for MAVROS/PX4 using AprilTag detections.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vision_pose_direct_node = drone_vision_pose_publisher.vision_pose_direct_node:main",
            "vision_pose_uxrce_node = drone_vision_pose_publisher.vision_pose_uxrce_node:main",
        ],
    },
)