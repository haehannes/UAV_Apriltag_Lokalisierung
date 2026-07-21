from setuptools import setup

package_name = 'drone_camera_calibration'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyyaml', 'numpy'],
    zip_safe=True,
    maintainer='Hannes Mueller',
    maintainer_email='hannesmueller99@t-onlinde.de',
    description='Berechnet gemittelte Transformationen zwischen Drohnenmittelpunkt und Kameras.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_camera_calibration_node = drone_camera_calibration.drone_camera_calibration_node:main',
        ],
    },
)