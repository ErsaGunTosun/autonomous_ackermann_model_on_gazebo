import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.15',
        description='ArUco marker boyutu (metre)'
    )
    
    show_debug_arg = DeclareLaunchArgument(
        'show_debug',
        default_value='true',
        description='Debug penceresi göster'
    )
    
    dictionary_id_arg = DeclareLaunchArgument(
        'dictionary_id',
        default_value='0',
        description='ArUco sözlük ID (0=DICT_4X4_50, 1=DICT_4X4_100, ...)'
    )
    
    aruco_detector_node = Node(
        package='vehicle_perception',
        executable='aruco_detector_node',
        name='aruco_detector_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'marker_size': LaunchConfiguration('marker_size'),
            'show_debug_window': LaunchConfiguration('show_debug'),
            'dictionary_id': LaunchConfiguration('dictionary_id'),
        }],
        remappings=[
            ('/camera', '/camera'),
        ]
    )

    return LaunchDescription([
        marker_size_arg,
        show_debug_arg,
        dictionary_id_arg,
        aruco_detector_node
    ])
