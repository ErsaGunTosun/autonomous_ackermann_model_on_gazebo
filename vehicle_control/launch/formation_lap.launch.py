import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    vehicle_nav_dir = get_package_share_directory('vehicle_navigation')
    vehicle_perception_dir = get_package_share_directory('vehicle_perception')
    
    navigation_formation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vehicle_nav_dir, "launch", "bring_up_nav_formation.launch.py")
        )
    )
    
    lane_controlloler_node = Node(
        package = "vehicle_control",
        executable="lane_controller_node",
        name='lane_controller_node',
        output='screen',
        parameters = [{
                "use_sim_time":True,
        }]
    )

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
    
    formation_lap_node = Node(
        package='vehicle_control',
        executable='formation_lap_node',
        name='formation_lap_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    data_logger_node = Node(
        package='vehicle_analysis',
        executable='data_logger_node',
        name='data_logger_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=formation_lap_node,
            on_exit=EmitEvent(event=Shutdown(reason='Formation lap completed'))
        )
    )

    return LaunchDescription([
        navigation_formation_launch,
        lane_controlloler_node,
        aruco_detector_node,
        formation_lap_node,
        data_logger_node,
        shutdown_on_exit,
    ])