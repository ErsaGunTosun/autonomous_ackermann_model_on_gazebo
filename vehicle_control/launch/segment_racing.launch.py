import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    vehicle_perception_dir = get_package_share_directory('vehicle_perception')
    
    aruco_detector_node = Node(
        package='vehicle_perception',
        executable='aruco_detector_node',
        name='aruco_detector_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'marker_size': 0.15,
            'show_debug_window': True,
            'dictionary_id': 0,
        }]
    )
    
    segment_racing_node = Node(
        package='vehicle_control',
        executable='segment_racing_node',
        name='segment_racing_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'total_laps': 3,
            'steering_kp': 0.8,
            'steering_kd': 0.1,
            'min_speed': 0.2,
            'max_speed': 2.0,
            'default_speed': 0.8,
            'speed_increase_rate': 1.10,
            'speed_decrease_rate': 0.80,
            'crash_threshold': 0.15,
            'start_box_radius': 1.5,
            'min_leave_distance': 3.0,
        }]
    )

    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=segment_racing_node,
            on_exit=EmitEvent(event=Shutdown(reason='Racing completed'))
        )
    )

    return LaunchDescription([
        aruco_detector_node,
        segment_racing_node,
        shutdown_on_exit,
    ])
