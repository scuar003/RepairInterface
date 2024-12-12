from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        #Transforms
        Node (
            package='tf2_ros',
            executable='static_transform_publisher',
            name='rsLidar_tf',
            arguments=['0.042', '0.0', '0.036', '1.570', '3.19', '-0.02', 'camera_link', 'camera_link'],
            output='screen'
        ),
        Node (
            package='tf2_ros',
            executable='static_transform_publisher',
            name='rpLidar_tf',
            arguments=['0.13', '0', '0', '0', '1.57', '0', 'tool0', 'laser'],
            output='screen'
        ),
        
        #UI Interface
        Node(
            package='demo_cpp_ready',
            executable='main_menu',
            output='screen'
        ),
        Node(
            package='demo_cpp_ready',
            executable='repair_interface',
            output='screen'
        ),
        
        #Mapping and Detection
        Node(
            package='demo_cpp_ready',
            executable='lidar_scan',
            output='screen'
        ),
        
        Node(
            package='demo_py_ready',
            executable='surface_detection',
            output='screen'
        ),
        #repair operations
        Node(
            package='demo_py_ready',
            executable='repair_grind',
            output='screen'
        ),
        Node(
            package='demo_py_ready',
            executable='repair_vacuum',
            output='screen'
        ),
        Node(
            package='demo_py_ready',
            executable='repair_expo_marker',
            output='screen'
        ),

        # #moves
        Node(
            package='move_cmd_py',
            executable='MoveHome',
            output='screen'
        ),
        Node(
            package='move_cmd_cpp',
            executable='move3Dmouse',
            output='screen'
        ),
    ])
