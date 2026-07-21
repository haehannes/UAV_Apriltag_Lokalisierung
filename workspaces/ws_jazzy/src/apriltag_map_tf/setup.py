import os
from glob import glob

from setuptools import find_packages, setup

package_name = "apriltag_map_tf"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hannes Mueller",
    maintainer_email="hannesmueller99@t-online.de",
    description="Publiziert eine AprilTag-Karte als statische TF und Marker fuer RViz.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "map_tf_node = apriltag_map_tf.map_tf_node:main",
        ],
    },
)