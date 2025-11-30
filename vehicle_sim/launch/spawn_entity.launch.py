import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.01')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0')
    
    spawn_entity = Node(
        package = "ros_gz_sim",
        executable = "create",
        arguments=[
            "-name","robot",
            "-topic","robot_description",
            "-x", LaunchConfiguration('x'),
            "-y", LaunchConfiguration('y'),
            "-z", LaunchConfiguration('z'),
            "-Y", LaunchConfiguration('yaw'),
        ],
        parameters=[
            {'use_sim_time': True}
        ],
        output="screen"
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/robot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/IMU@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/depth_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
        ('/model/robot/odometry', '/odom'),
        ]
    )


    return LaunchDescription([
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        spawn_entity,
        bridge
    ])