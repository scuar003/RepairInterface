from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
            executable='task_action',
            output='screen'
        ),
        Node(
            package='demo_py_ready',
            executable='surface_detection',
            output='screen'
        ),
        Node(
            package='demo_py_ready',
            executable='repair_executer',
            output='screen'
        ),
    ])
