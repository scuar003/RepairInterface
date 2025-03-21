import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from pyfirmata import Arduino
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

def get_rotation_angle(v1, v2):
    return np.arccos(np.dot(v1, v2))

# Move the robot to its home position
def home(robot, acc, vel):
    home_position = (deg2rad(-90), deg2rad(-90), deg2rad(-90), deg2rad(-90), deg2rad(90), deg2rad(0))
    robot.movej(home_position, acc, vel)

# Calculate the orientation for vacuuming based on the points
def vector_to_euler_angles(target_normal):
   initial_vector = np.array([0, 0, 1])
   target_normal = normalize(target_normal)
   rotation_axis = np.cross(initial_vector, target_normal)
   rotation_axis_normalized = normalize(rotation_axis)
   cos_angle = np.dot(initial_vector, target_normal)
   angle = np.arccos(cos_angle)
   qx = rotation_axis_normalized[0] * np.sin(angle / 2)
   qy = rotation_axis_normalized[1] * np.sin(angle / 2)
   qz = rotation_axis_normalized[2] * np.sin(angle / 2)
   qw = np.cos(angle / 2)
   # Using a 'zyx' rotation order
   roll = np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx**2 + qy**2))
   pitch = np.arcsin(2 * (qw * qy - qz * qx))
   yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))
   return roll, pitch, yaw

def getVacuum(robot, tool_changer, unlock, lock, vacuum_payload, vacuum_tcp, vacuum_cog):
    home(robot, 0.5, 0.5)
    robot.set_tcp((0,0,0,0,0,0))
    tool_changer.write(unlock)
    robot.movel((0.42244, 0.09685, 0.43866, 0, 3.143, -0.000), 0.3, 0.3)
    robot.movel((0.42244, 0.09684, 0.28188, 0, 3.143, -0.000), 0.3, 0.3)
    robot.movel((0.42243, 0.09688, 0.22548, 0, 3.143, -0.000), 0.1, 0.1)
    time.sleep(0.2)  
    tool_changer.write(lock)
    time.sleep(0.2)
    robot.set_payload(vacuum_payload, vacuum_cog)
    time.sleep(0.2)
    robot.movel((0.42243, 0.09685, 0.27521, 0, 3.143, -0.000), 0.2, 0.2)
    robot.movel((0.43839, 0.09686, 0.27521, 0, 3.143, -0.000), 0.3, 0.3)
    robot.movel((0.43839, -0.06370, 0.27521, 0, 3.143, -0.000), 0.3, 0.3)
    robot.movel((0.43839, -0.06370, 0.48735, 0, 3.143, -0.000), 0.3, 0.3)
    home(robot, 0.5, 0.5)
    time.sleep(0.2)
    robot.set_tcp(vacuum_tcp)
    time.sleep(0.2)
    
def returnVacuum(robot, tool_changer, unlock, normal_payload, normal_tcp):
    home(robot, 0.5, 0.5)
    robot.set_tcp(normal_tcp)
    robot.movel((0.43839, -0.06370, 0.48735, 0, 3.143, -0.000), 0.3, 0.3)
    robot.movel((0.43839, -0.06370, 0.27521, 0, 3.143, -0.000), 0.3, 0.3)
    robot.movel((0.43839, 0.09686, 0.27521, 0, 3.143, -0.000), 0.3, 0.3)
    robot.movel((0.42243, 0.09685, 0.27521, 0, 3.143, -0.000), 0.2, 0.2)
    robot.movel((0.42243, 0.09688, 0.22548, 0, 3.143, -0.000), 0.1, 0.1)
    time.sleep(0.2)
    tool_changer.write(unlock)
    time.sleep(0.2)
    robot.set_payload(normal_payload)
    time.sleep(0.2)
    robot.movel((0.42244, 0.09684, 0.28188, 0, 3.143, -0.000), 0.2, 0.2)
    robot.movel((0.42244, 0.09685, 0.43866, 0, 3.143, -0.000), 0.3, 0.3)
    home(robot, 0.5, 0.5)

def offset(corner, offset, normal):
   corner_new = corner + offset*normal
   return corner_new

def rotate(robot, angle):
    o = robot.get_orientation()
    o.rotate_zb(angle)
    robot.set_orientation(o, 0.2, 0.2)

def follow_path(robot, points, acc, vel, normal_vector):
    p1 = points[0]
    p2 = points[1]
    p3 = points[2]
    p4 = points[3]
    grid_size = 0.01
    move_vector = p2 - p1
    shift_vector = normalize(p4 - p1) * grid_size
    num_passes = int(np.linalg.norm(p4 - p1) / grid_size) + 1
    rz_rotation = np.pi
    current_position = p1.copy()
    temp_pos = robot.getl()
    robot.movel((current_position[0], current_position[1], current_position[2], temp_pos[3], temp_pos[4], temp_pos[5]), acc, vel)
    for pass_num in range(num_passes):
        move_end = current_position + move_vector
        # rotate(robot, -rz_rotation)
        time.sleep(0.2)
        temp_pos = robot.getl()
        robot.movel((move_end[0], move_end[1], move_end[2], temp_pos[3], temp_pos[4], temp_pos[5]), acc, vel)
        if pass_num < num_passes - 1:
            next_start_shifted = current_position + shift_vector
            # rotate(robot, rz_rotation)
            time.sleep(0.2)
            temp_pos = robot.getl()
            robot.movel((next_start_shifted[0], next_start_shifted[1], next_start_shifted[2], temp_pos[3], temp_pos[4], temp_pos[5]), acc, vel)
            current_position = next_start_shifted
    last_point = offset(move_end, 0.05, normal_vector)
    final_move = (last_point[0], last_point[1], last_point[2], temp_pos[3], temp_pos[4], temp_pos[5])
    robot.movel(final_move, acc, vel)

# Perform the vacuum task
def vacuum(ur_control, acc, vel, normal_vector, points, tool, tool_changer):
    home(ur_control.robot, 0.5, 0.5)
    lock = 0
    unlock = 1
    tool_off = 0 
    tool_on = 1
    vacuum_payload = 2.230
    vacuum_cog = (-0.012, -0.010, 0.098)
    normal_payload = 1.100
    normal_tcp = (0, 0, 0, 0, 0, 0)
    vacuum_tcp = (-0.05199, 0.18001, 0.22699, 2.4210, -0.0025, 0.0778)
    getVacuum(ur_control.robot, tool_changer, unlock, lock, vacuum_payload, vacuum_tcp, vacuum_cog)
    ur_control.robot.movej((-1.57, -1.57, -1.57, -1.57, 1.57, 3.14), 0.5, 0.5)
    ur_control.robot.set_payload(vacuum_payload)
    ur_control.robot.set_tcp(vacuum_tcp)
    linearPosition = ur_control.robot.getl()
    linearPosition[1] = -0.400 
    linearPosition[0] = -0.400
    ur_control.robot.movel(linearPosition, 0.2, 0.2)
    ur_control.robot.movel(linearPosition[:3]+[0,0,0], 0.2, 0.2)
    normal_vector = normalize(np.cross(points[1] - points[0], points[2] - points[0]))
    if normal_vector[2] < 0:
        normal_vector = -normal_vector
    orientation = vector_to_euler_angles(normal_vector)
    eax = orientation[0]
    eay = orientation[1]
    eaz = orientation[2]
    o = ur_control.robot.get_orientation()
    o.rotate_xb(eax)
    o.rotate_yb(eay)
    o.rotate_zb(eaz)
    ur_control.robot.set_orientation(o)
    time.sleep(0.2)
    tool.write(tool_on)
    follow_path(ur_control.robot, points, acc, vel, normal_vector)
    tool.write(tool_off)
    ur_control.robot.set_tcp(normal_tcp)
    home(ur_control.robot, 0.5, 0.5)
    returnVacuum(ur_control.robot, tool_changer, unlock, normal_payload, normal_tcp)
    ur_control.robot.set_payload(normal_payload)
    # Clean
    ur_control.clear_path()

# ROS2 Node for controlling the UR robot
class URControlNode(Node):
    def __init__(self):
        super().__init__('ur_control_node')
        self.subscription = self.create_subscription(PoseArray, 'repair_area/vacum', self.pose_array_callback, 10)
        self.marker_publisher = self.create_publisher(Marker,'repair_path', 10)
        self.points_list = []
        self.robot_ip = "172.16.3.131"  # Replace with your robot's IP address
        self.robot = None

    def pose_array_callback(self, msg):
        # Saving corners
        self.clear_path()
        self.points_list.clear()
        for pose in msg.poses:
            processed_point = np.array([-round(pose.position.x , 2 ), 
                                        -round(pose.position.y , 2 ), 
                                         round(pose.position.z , 2 )])
            self.points_list.append(processed_point)
        # Make sure there are four corners
        if len(self.points_list) >= 4:
            self.points_list = self.points_list[-4:]
            self.control_ur_robot()

    def control_ur_robot(self):
        connected = False
        tries = 0
        maxTries= 5
        while not connected and tries < maxTries:
            try:
                time.sleep(0.3)
                robot = urx.Robot(self.robot_ip)
                time.sleep(0.3)
                connected = True
            except:
                tries += 1
                print(f"Connection attempt {tries} failed.")
                time.sleep(1)  # Wait for a second before next attempt
        if connected:
            points = np.array(self.points_list)
            self.robot = urx.Robot(self.robot_ip)
            board = Arduino('/dev/ttyACM0')
            tool_relay_pin_number = 7
            tool = board.get_pin(f'd:{tool_relay_pin_number}:o')
            tool_changer_relay_pin_number = 8
            tool_changer = board.get_pin(f'd:{tool_changer_relay_pin_number}:o')
            normal_vector = []
            home(self.robot, 0.8, 0.8)
            vacuum(self, 0.2, 0.2, normal_vector, points, tool, tool_changer)
            home(self.robot,0.8,0.8)
            self.robot.close()
            self.robot = None
        else:
            print("Connection failed. Check robot state.")

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