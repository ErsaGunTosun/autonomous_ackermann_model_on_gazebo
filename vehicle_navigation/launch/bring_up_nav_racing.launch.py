import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Racing için Navigation Launch
    Sadece Nav2 başlatır (statik harita ile)
    SLAM başlatmaz
    Map Server ile kaydedilen haritayı yükler
    """
    vehicle_nav_dir = get_package_share_directory('vehicle_navigation')
    nav2_pkg_dir = get_package_share_directory('nav2_bringup')
    
    # Map dosyası yolu - launch argümanı olarak
    map_file_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(vehicle_nav_dir, 'maps', 'formation_map.yaml'),
        description='Full path to map yaml file to load'
    )
    map_file = LaunchConfiguration('map')
    
    rviz_config_file = os.path.join(vehicle_nav_dir, 'config', 'rviz_config.rviz')
    nav2_params_file = os.path.join(vehicle_nav_dir, 'config', 'nav2_params_ackermann.yaml')
    
    # Map Server (statik harita yükleme)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': map_file
        }]
    )
    
    # Nav2 Navigation Stack (statik harita ile)
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
    
    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name='rviz2',
        arguments=['-d', rviz_config_file, '--ros-args', '--log-level', 'warn'],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        map_file_arg,
        map_server_node,
        nav2_launch,
        rviz_node,
    ])

