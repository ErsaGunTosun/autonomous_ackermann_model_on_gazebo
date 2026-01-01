import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """
    Formation Lap Launch
    - Navigation (SLAM - harita çıkarmak için)
    - Formation lap node (tur takibi ve harita kaydetme)
    """
    vehicle_nav_dir = get_package_share_directory('vehicle_navigation')
    vehicle_cntrl_dir = get_package_share_directory('vehicle_control')
    
    navigation_formation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vehicle_nav_dir,"launch","bring_up_nav_formation.launch.py")
        )
    )
    
    formation_lap_node = Node(
        package='vehicle_control',
        executable='formation_lap_node',
        name='formation_lap_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Formation lap node kapandığında launch'ı da durdur
    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=formation_lap_node,
            on_exit=EmitEvent(event=Shutdown(reason='Formation lap completed'))
        )
    )

    return LaunchDescription([
        navigation_formation_launch,
        formation_lap_node,
        shutdown_on_exit,
    ])

