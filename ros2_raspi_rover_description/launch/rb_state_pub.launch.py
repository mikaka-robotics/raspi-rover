from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open('/home/turtlebot/ros2_ws/src/ros2_raspi_rover_description/src/description/ros2_raspi_rover_description.urdf', 'r').read()}]
        )
    ])
