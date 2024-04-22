
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
    home(robot)
    robot.set_tcp((0,0,0,0,0,0))
    tool_changer.write(unlock)
    robot.movel((0.25525, -0.69405, 0.29491, 0, 3.141, -0.007), 0.7, 0.7)
    robot.movel((0.25525, -0.69407, 0.07763, 0, 3.141, -0.007), 0.7, 0.7)
    robot.movel((0.25521, -0.69403, 0.00607, 0.001, 3.141, -0.006), 0.1, 0.1)
    time.sleep(0.2)  
    tool_changer.write(lock)
    time.sleep(0.2)
    robot.set_payload(vacuum_payload, vacuum_cog)
    time.sleep(0.2)
    robot.movel((0.25525, -0.69407, 0.07763, 0, 3.141, -0.007), 0.2, 0.2)
    robot.movel((0.25525, -0.69405, 0.29491, 0, 3.141, -0.007), 0.7, 0.7)
    home(robot)
    time.sleep(0.2)
    robot.set_tcp(vacuum_tcp)
    time.sleep(0.2)
    
def returnVacuum(robot, tool_changer, unlock, normal_payload, normal_tcp):
    home(robot)
    robot.set_tcp(normal_tcp)
    robot.movel((0.25525, -0.69405, 0.29491, 0, 3.141, -0.007), 0.7, 0.7)
    robot.movel((0.25525, -0.69407, 0.07763, 0, 3.141, -0.007), 0.7, 0.7)
    robot.movel((0.25521, -0.69403, 0.00607, 0.001, 3.141, -0.006), 0.1, 0.1) 
    time.sleep(0.2)
    tool_changer.write(unlock)
    time.sleep(0.2)
    robot.set_payload(normal_payload)
    time.sleep(0.2)
    robot.movel((0.25525, -0.69407, 0.07763, 0, 3.141, -0.007), 0.2, 0.2)
    robot.movel((0.25525, -0.69405, 0.29491, 0, 3.141, -0.007), 0.7, 0.7)
    home(robot)

def offset(corner, offset, normal):
   corner_new = corner - offset*normal
   return corner_new

def calculate_rotations(path):
    rotations = []
    for counter in range(len(path) - 2):
        initial_dir_vector =  normalize(np.array(path[counter + 1]) - np.array(path[counter]))
        final_dir_vector = normalize(np.array(path[counter + 2]) - np.array(path[counter + 1]))
        angle_of_rotation = get_rotation_angle(initial_dir_vector, final_dir_vector)
        rotations.append(angle_of_rotation)
    return rotations

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
    # getVacuum(robot, tool_changer, unlock, lock, vacuum_payload, vacuum_tcp, vacuum_cog)
    ur_control.robot.movej((-1.57, -1.57, -1.57, -1.57, 1.57, 3.14), 0.5, 0.5)
    ur_control.robot.set_payload(vacuum_payload)
    ur_control.robot.set_tcp(vacuum_tcp)
    linearPosition = ur_control.robot.getl()
    linearPosition[1] = -0.400 
    linearPosition[0] = -0.400
    ur_control.robot.movel(linearPosition, 0.2, 0.2)
    ur_control.robot.movel(linearPosition[:3]+[0,0,0], 0.2, 0.2)
    orientation = vector_to_euler_angles(normal_vector)
    eax = orientation[0]
    eay = orientation[1]
    eaz = orientation[2]
    o = ur_control.robot.get_orientation()
    o.rotate_xb(eax)
    o.rotate_yb(eay)
    o.rotate_zb(eaz)
    ur_control.robot.set_orientation(o)
    linearPosition = ur_control.robot.getl() 
    rx = linearPosition[3]
    ry = linearPosition[4]
    rz = linearPosition[5]
    path = points
    last_point = offset(points[-1], 0.01, normal_vector)
    # tool.write(tool_on)
    rotations = calculate_rotations(path)
    counter = 0
    while counter < len(path):
        temp_pos = ur_control.robot.getl()
        if counter == 0:
            target = path[counter] + temp_pos[-3:]
            ur_control.robot.movel(target, acc, vel)
            temp_pos = ur_control.robot.getl()
            temp_list = temp_pos[:3]
            temp_v1 = normalize(np.array([0,1,0]))
            temp_v2 = normalize(np.array(temp_list))
            o = ur_control.robot.get_orientation() 
            o.rotate_zb(get_rotation_angle(temp_v1, temp_v2))
            ur_control.robot.set_orientation(o, 0.3, 0.3)
        elif counter == len(path) - 1:
            target = path[counter] + temp_pos[-3:]
            ur_control.robot.movel(target, acc, vel)
        else:
            target = path[counter] + temp_pos[-3:]
            ur_control.robot.movel(target, acc, vel)
            o = ur_control.robot.get_orientation() 
            o.rotate_zb(rotations[counter-1])
            ur_control.robot.set_orientation(o, 0.3, 0.3)    
        counter = counter + 1    
    temp_pos = ur_control.robot.getl()
    target = [temp_pos[0], temp_pos[1], temp_pos[2]]
    last_point = offset(target, 0.05, normal_vector)
    final_move = (last_point[0], last_point[1], last_point[2], temp_pos[3], temp_pos[4], temp_pos[5])
    ur_control.robot.movel(final_move, acc, vel)
    # tool.write(tool_off)
    ur_control.robot.set_tcp(normal_tcp)
    home(ur_control.robot, 0.5, 0.5)
    # returnVacuum(robot, tool_changer, unlock, normal_payload, normal_tcp)
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
        self.robot_ip = "172.16.0.9"  # Replace with your robot's IP address
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
                robot = urx.Robot("172.16.0.9")
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
            vacuum(self, 0.3, 0.3, normal_vector, points, tool, tool_changer)
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
