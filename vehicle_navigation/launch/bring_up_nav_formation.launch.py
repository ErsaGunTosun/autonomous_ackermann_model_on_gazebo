import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    """
    Formation Lap için Navigation Launch
    Sadece SLAM Toolbox başlatır (harita çıkarmak için)
    Nav2 başlatmaz
    """
    vehicle_nav_dir = get_package_share_directory('vehicle_navigation')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    
    rviz_config_file = os.path.join(vehicle_nav_dir, 'config', 'rviz_config.rviz')
    slam_config_file = os.path.join(vehicle_nav_dir, 'config', 'slam_toolbox_params.yaml')
    
    # SLAM Toolbox (sadece harita çıkarmak için)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': slam_config_file,
        }.items()
    )
    
    # RViz (harita görselleştirme için)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name='rviz2',
        arguments=['-d', rviz_config_file, '--ros-args', '--log-level', 'warn'],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        slam_launch,
        rviz_node,
    ])

