import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    vehicle_nav_dir = get_package_share_directory('vehicle_navigation')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    nav2_pkg_dir = get_package_share_directory('nav2_bringup')
    
    rviz_config_file = os.path.join(vehicle_nav_dir, 'config', 'rviz_config.rviz')
    slam_config_file = os.path.join(vehicle_nav_dir, 'config', 'slam_toolbox_params.yaml')
    nav2_params_file = os.path.join(vehicle_nav_dir, 'config', 'nav2_params_ackermann.yaml')
    
    # SLAM Toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': slam_config_file,
        }.items()
    )
    
    # Nav2 Navigation Stack
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params_file,
            'autostart': 'true',
        }.items()
    )
    
    # Nav2'yi SLAM başladıktan 5 saniye sonra başlat
    delayed_nav2 = TimerAction(
        period=5.0,
        actions=[nav2_launch]
    )
    
    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name='rviz2',
        arguments=['-d', rviz_config_file, '--ros-args', '--log-level', 'warn'],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        slam_launch,
        delayed_nav2,
        rviz_node,
    ])
