import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """
    Racing Lap Launch
    - Navigation (Map Server, AMCL, Nav2)
    - Racing node (3 tur waypoint takibi)
    """
    vehicle_nav_dir = get_package_share_directory('vehicle_navigation')
    vehicle_cntrl_dir = get_package_share_directory('vehicle_control')
    
    # Racing Navigation Launch (Map Server, AMCL, Nav2)
    navigation_racing_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vehicle_nav_dir, "launch", "bring_up_nav_racing.launch.py")
        )
    )
    
    # Racing Node
    racing_node = Node(
        package='vehicle_control',
        executable='racing_node',
        name='racing_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Racing node kapandığında launch'ı da durdur
    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=racing_node,
            on_exit=EmitEvent(event=Shutdown(reason='Racing completed'))
        )
    )

    return LaunchDescription([
        navigation_racing_launch,
        racing_node,
        shutdown_on_exit,
    ])
