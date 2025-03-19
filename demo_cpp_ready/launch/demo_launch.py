from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        #Transforms
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='camera_link_transform',
        #     arguments=[
        #         '--x', '0.042', '--y', '0.0', '--z', '0.036',
        #         '--qx', '0.4250795', '--qy', '0.8636966', '--qz', '-0.005415', '--qw', '0.2707513',
        #         '--frame-id', 'camara_link', '--child-frame-id', 'camera_link'
        #     ],
        #     output='screen'
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='tool0_to_laser_transform',
        #     arguments=[
        #         '--x', '0.13', '--y', '0', '--z', '0',
        #         '--qx', '0', '--qy', '0.8434402', '--qz', '0', '--qw', '0.5372231',
        #         '--frame-id', 'tool0', '--child-frame-id', 'laser'
        #     ],
        #     output='screen'
        # ),
        
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
