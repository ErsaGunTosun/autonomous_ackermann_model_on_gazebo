import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    lane_follow_node = Node(
        package = "vehicle_perception",
        executable="lane_follow_node",
        name='lane_follow_node',
        output='screen',
        parameters = [{
                "use_sim_time":True,
        }]
    )

    return LaunchDescription([
        lane_follow_node
    ])