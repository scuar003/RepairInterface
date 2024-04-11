#!/bin/bash


ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{activate_controllers: ['forward_position_controller'], deactivate_controllers: ['scaled_joint_trajectory_controller'], strictness: 1}"
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}
ros2 service call /dashboard_client/play std_srvs/srv/Trigger