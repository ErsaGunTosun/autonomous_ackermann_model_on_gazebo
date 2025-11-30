import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
import subprocess

def generate_launch_description():
    
    pkg_share = get_package_share_directory("vehicle_sim")
    robot_path = os.path.join(
            get_package_share_directory("vehicle_sim"),
            "urdf",
            "ackermann.urdf.xacro"
    )

    robot_desc = subprocess.check_output([
        "xacro", robot_path
    ]).decode('utf-8')
    
    robot_desc = robot_desc.replace(
            'package://vehicle_sim',
            f'file://{pkg_share}'
    )
    robot_state_publisher = Node(
            package = "robot_state_publisher",
            executable="robot_state_publisher",
            parameters = [{
                "use_sim_time":True,
                "robot_description":robot_desc
                }]
    )

    return LaunchDescription([
        robot_state_publisher
    ])
