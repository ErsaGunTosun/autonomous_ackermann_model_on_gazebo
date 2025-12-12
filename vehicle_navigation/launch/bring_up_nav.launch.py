import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    vehicle_nav_dir = get_package_share_directory('vehicle_navigation')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    rviz_config_file = os.path.join(vehicle_nav_dir,'config', 'rviz_config.rviz')
    
    slam_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_dir,"launch","online_async_launch.py")
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'log_level': 'error'
            }.items()
        )
    
    rviz_node = Node(
        package = "rviz2",
        executable="rviz2",
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
       slam_launch,
       rviz_node
    ])