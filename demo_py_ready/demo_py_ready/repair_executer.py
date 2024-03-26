import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

import urx
import numpy as np
import time

# Convert degrees to radians
def deg2rad(deg):
    return deg * np.pi / 180

# Normalize a vector
def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0: 
        return v
    return v / norm

# Move the robot to its home position
def home(robot, acc, vel):
    home_position = (deg2rad(-90), deg2rad(-90), deg2rad(-90), deg2rad(-90), deg2rad(90), deg2rad(0))
    robot.movej(home_position, acc, vel)

# Convert a rotation matrix to a rotation vector
def rotation_matrix_to_rotation_vector(rot):
    theta = np.arccos((np.trace(rot) - 1) / 2)
    if theta != 0:
        rx = (rot[2,1] - rot[1,2]) / (2 * np.sin(theta))
        ry = (rot[0,2] - rot[2,0]) / (2 * np.sin(theta))
        rz = (rot[1,0] - rot[0,1]) / (2 * np.sin(theta))
        return np.array([rx, ry, rz]) * theta
    else:
        return np.array([0, 0, 0])

# Calculate the orientation for grinding based on the points
def getOrientation(points):
    p1 = points[0]
    p2 = points[1]
    p4 = points[3]
    v1 = p2 - p1
    v2 = p4 - p1
    z_axis = normalize(np.cross(v1, v2))
    x_axis = normalize(np.cross([0, 0, -1], z_axis))
    if np.linalg.norm(x_axis) == 0:
        x_axis = normalize(np.cross([0, 1, 0], z_axis))
    y_axis = normalize(np.cross(z_axis, x_axis))
    rot = np.array([x_axis, y_axis, z_axis]).T
    rotation_vector = rotation_matrix_to_rotation_vector(rot)
    return rotation_vector

# Generate waypoints for grinding
def offset(corner, offset, normal):
   corner_new = corner - offset*normal
   return corner_new

def generateWaypoints(grid_size, lift_distance, lower, rx, ry, rz, points):
   op1, op2, op3, op4 = points
   normal_vector = normalize(np.cross(op2 - op1, op3 - op1))
   p1 = offset(op1, lower, normal_vector)
   p2 = offset(op2, lower, normal_vector)
   p3 = offset(op3, lower, normal_vector)
   p4 = offset(op4, lower, normal_vector)
   normal_vector = normalize(np.cross(p2 - p1, p3 - p1))
   move_vector = p2 - p1
   shift_vector = normalize(p4 - p1) * grid_size
   num_passes = int(np.linalg.norm(p4 - p1) / grid_size) + 1
   waypoints = []
   current_position = p1.copy()
   for pass_num in range(num_passes):
       move_end = current_position + move_vector
       waypoints.append((current_position[0], current_position[1], current_position[2], rx, ry, rz))
       waypoints.append((move_end[0], move_end[1], move_end[2], rx, ry, rz))
       lifted_position = move_end + np.array([0, 0, lift_distance])
       waypoints.append((lifted_position[0], lifted_position[1], lifted_position[2], rx, ry, rz))
       if pass_num < num_passes - 1:
           next_start_at_lifted_height = current_position + shift_vector + np.array([0, 0, lift_distance])
           waypoints.append((next_start_at_lifted_height[0], next_start_at_lifted_height[1], next_start_at_lifted_height[2], rx, ry, rz))
           waypoints.append((next_start_at_lifted_height[0], next_start_at_lifted_height[1], next_start_at_lifted_height[2], rx, ry, rz))
           next_start_lowered = next_start_at_lifted_height - np.array([0, 0, lift_distance])
           waypoints.append((next_start_lowered[0], next_start_lowered[1], next_start_lowered[2], rx, ry, rz))
           current_position = next_start_lowered
   return waypoints

# Perform the grinding task
def grindSurface(ur_control, acc, vel,  numPasses, points):
   home(ur_control.robot, 0.5, 0.5)
   waypoints = []
   gridSize = 0.01
   liftDistance = 0.01
#    orientation = getOrientation(points)
   rx = 0
   ry = 0
   rz = 0
   if rx == 0 and ry == 0 and rz == 0:
       rx = 0
       ry = 3.14
       rz = 0
   waypoints = []
   tempx = []
   tempy = []
   PASSES = 2
   GRIND_ANGLE = 0.420
   count = 0
   lowerDistance = 0.005
   while count <= numPasses - 1:
       if count == 0:
           waypoints = generateWaypoints(gridSize, liftDistance, 0 * count, rx, ry, rz + GRIND_ANGLE, points)
       else:
           waypoints = generateWaypoints(gridSize, liftDistance, lowerDistance * count, rx, ry, rz + GRIND_ANGLE, points)
       ur_control.show_path(waypoints) 
       passOver = 0
       while passOver <= PASSES - 1:
           for x in waypoints:
               ur_control.robot.movel(x,acc,vel)
               tempx.append(x[0])
               tempy.append(x[1])
           passOver = passOver+ 1
           print("we are on passover: ", passOver)
       count = count + 1
       print("We are on counter: ", count)
       # Clean
       ur_control.clear_path()

# ROS2 Node for controlling the UR robot
class URControlNode(Node):
    def __init__(self):
        super().__init__('ur_control_node')
        self.subscription = self.create_subscription(PoseArray, 'repair_area_corners', self.pose_array_callback, 10)
        self.marker_publisher = self.create_publisher(Marker,'repair_path', 10)
        self.points_list = []
        self.robot_ip = "172.16.3.114"  # Replace with your robot's IP address
        self.robot = None

    def pose_array_callback(self, msg):
        # Saving corners
        self.clear_path()
        self.points_list.clear()
        for pose in msg.poses:
            processed_point = np.array([-round(pose.position.x, 2), 
                                        -round(pose.position.y, 2), 
                                         round(pose.position.z, 2)])
            self.points_list.append(processed_point)
        # Make sure there are four corners
        if len(self.points_list) >= 4:
            self.points_list = self.points_list[-4:]
            self.control_ur_robot()

    def control_ur_robot(self):
        points = np.array(self.points_list)
        self.robot = urx.Robot(self.robot_ip)
        home(self.robot, 0.8, 0.8)
        removemm = 2
        numPasses = np.floor(removemm / 0.5)
        home(self.robot,0.8,0.8)
        grindSurface(self, 0.1, 0.1, numPasses, points)
        home(self.robot,0.8,0.8)
        self.robot.close()
        self.robot = None

    def show_path(self, waypoints):
        marker = Marker() 
        marker.id = 1 
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "base_link"
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.001
        # Color    
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        # Geometry
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        # Waypoints
        marker.points = []
        for point in waypoints:
            p = Point() 
            p.x = -point[0]
            p.y = -point[1]
            p.z = point[2]
            marker.points.append(p)
        # Publish
        self.marker_publisher.publish(marker)

    def clear_path(self):
        marker = Marker() 
        marker.id = 1 
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "base_link"
        marker.action = Marker.DELETE
        self.marker_publisher.publish(marker)

def main():
    rclpy.init()
    ur_control_node = URControlNode()
    rclpy.spin(ur_control_node)
    ur_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()