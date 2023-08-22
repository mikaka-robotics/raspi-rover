import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='ros2_raspi_rover_description').find('ros2_raspi_rover_description')
    default_model_path = os.path.join(pkg_share, 'src/description/ros2_raspi_rover_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/nav2_default_view.rviz')
    world_path=os.path.join(pkg_share, 'world/turtlebot3_world.world')
    map_path = os.path.join(pkg_share, "maps", 'sim_map.yaml')
    nav_params_path = os.path.join(pkg_share, "config", 'nav2_params.yaml')
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    spawn_entity = launch_ros.actions.Node(
    	package='gazebo_ros', 
    	executable='spawn_entity.py',
        arguments=['-entity', 'ros2_raspi_rover', '-topic', 'robot_description', '-x', '-2.0', '-y', '-0.5'],
        output='screen'
    )
    robot_localization_node = launch_ros.actions.Node(
         package='robot_localization',
         executable='ekf_node',
         name='ekf_filter_node',
         output='screen',
         parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
        launch_arguments={
            'map': map_path,
            'use_sim_time': use_sim_time,
            'params_file': nav_params_path}.items(),
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        robot_localization_node,
        bringup_launch,
        rviz_node
    ])