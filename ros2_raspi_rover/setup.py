from setuptools import setup

package_name = 'ros2_raspi_rover'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='dog.vrn0224@gmail.com',
    description='control 2-wheel differential robot',
    license='Apache Licence 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twist_to_radsec = ros2_raspi_rover.twist_to_radsec_diff:main',
            'twist_to_odom = ros2_raspi_rover.twist_to_odom:main',
            'twist_to_websocket = ros2_raspi_rover.twist_to_websocket:main',
            'odom_to_joint_state = ros2_raspi_rover.odom_to_joint_state:main'
        ],
    },
)
