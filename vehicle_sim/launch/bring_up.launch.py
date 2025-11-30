from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler, EmitEvent, ExecuteProcess, TimerAction
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit
import os

def generate_launch_description():
    vehicle_sim_dir = get_package_share_directory('vehicle_sim')
    ros_gz_sim = get_package_share_directory("ros_gz_sim")

    world_path = os.path.join(vehicle_sim_dir, 'worlds', 'empty_world.world')

    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim,"launch","gz_sim.launch.py")
        ),
        launch_arguments ={"gz_args": ["-s -r -v1 ", world_path]}.items()
    )

    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim,"launch","gz_sim.launch.py")    
        ),
        launch_arguments = {"gz_args": ["-g -v1"]}.items()
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vehicle_sim_dir,"launch","robot_state_publisher.launch.py")
        )
    )

    spawn_entity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                os.path.join(vehicle_sim_dir,"launch","spawn_entity.launch.py")
            ),
            launch_arguments={
                'x': "0",
                'y': "0",
                'yaw': "0"
            }.items()
    )


    return LaunchDescription([
        gzserver_launch,
        gzclient_launch,
        robot_state_publisher,
        spawn_entity,
    ])