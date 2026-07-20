from setuptools import find_packages, setup

package_name = "control"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/control_console.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hannes",
    maintainer_email="hannes@example.com",
    description="Small console control application for MAVROS and uXRCE-DDS.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "control_console = control.control_console:main",
            "control_console_uxrce = control.control_console_uxrce:main",
        ],
    },
)