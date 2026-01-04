import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_share = get_package_share_directory('vehicle_sim')
    sdf_file = os.path.join(pkg_share, 'models', 'static_box', 'model.sdf')
    
    obstacles = [
        {'name': 'obstacle_1', 'x': '33.0', 'y': '3.0', 'z': '0.0'},
        {'name': 'obstacle_2', 'x': '-21.5', 'y': '-2.5', 'z': '0.0'},
        {'name': 'obstacle_3', 'x': '2.0', 'y': '8.0', 'z': '0.0'},
    ]
    
    spawn_nodes = []
    
    for obs in obstacles:
        spawn_nodes.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', obs['name'],
                    '-file', sdf_file,
                    '-x', obs['x'],
                    '-y', obs['y'],
                    '-z', obs['z'],
                ],
                parameters=[{'use_sim_time': True}],
                output='screen'
            )
        )
    
    return LaunchDescription(spawn_nodes)
