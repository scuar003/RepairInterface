import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ur_robot_driver_share_dir = get_package_share_directory('ur_robot_driver')
    ur_control_launch_file = os.path.join(ur_robot_driver_share_dir, 'launch', 'ur_control.launch.py')

    # Include the ur_control launch file and pass the required parameters
    include_ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_control_launch_file),
        launch_arguments={
            'ur_type': 'ur16e',
            'robot_ip': '192.168.56.101',
            'launch_rviz': 'true'
        }.items()
    )

        # Static transform publisher for camara_link -> camera_link
    static_tf_camara = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_camara',
        output='screen',
        arguments=['0.042', '0.0', '0.036', '1.570', '3.19', '-0.02', 'camara_link', 'camera_link']
    )

    # Static transform publisher for tool0 -> lase
    static_tf_tool0 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_tool0',
        output='screen',
        arguments=['-0.13', '-0.03', '0.04', '0', '1.57', '3.14', 'tool0', 'lase']
    )

    return LaunchDescription([
        include_ur_control,
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
        static_tf_camara,
        static_tf_tool0
    ])
