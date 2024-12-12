import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback
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

# Move the robot to its home position
def home(robot, acc, vel):
    home_position = (deg2rad(-90), deg2rad(-90), deg2rad(-90), deg2rad(-90), deg2rad(90), deg2rad(0))
    robot.movej(home_position, acc, vel)

# Convert a rotation matrix to a rotation vector
def getMarker(robot, tool_changer, unlock, lock, marker_payload, marker_tcp):
    home(robot, 0.5, 0.5)
    robot.set_tcp((0,0,0,0,0,0))
    tool_changer.write(unlock)
    robot.movel((0.26055, -0.03720, 0.48702, 2.204, 2.247, -0.067), 0.3, 0.3)
    robot.movel((0.26055, -0.03720, 0.28893, 2.204, 2.247, -0.067), 0.3, 0.3)
    robot.movel((0.26055, -0.03720, 0.23795, 2.204, 2.247, -0.067), 0.05, 0.05)
    time.sleep(0.2)  
    tool_changer.write(lock)
    time.sleep(0.2)
    robot.set_payload(marker_payload)
    time.sleep(0.2)
    robot.movel((0.26055, -0.03720, 0.28893, 2.204, 2.247, -0.067), 0.1, 0.1)
    robot.movel((0.26055, -0.03720, 0.48702, 2.204, 2.247, -0.067), 0.3, 0.3)
    home(robot, 0.5, 0.5)
    robot.set_tcp(marker_tcp)
    time.sleep(0.2)
    
def returnMarker(robot, tool_changer, unlock, normal_payload, normal_tcp):
    home(robot, 0.5, 0.5)
    robot.set_tcp(normal_tcp)
    robot.movel((0.26055, -0.03720, 0.48702, 2.204, 2.247, -0.067), 0.3, 0.3)
    robot.movel((0.26055, -0.03720, 0.28893, 2.204, 2.247, -0.067), 0.3, 0.3)
    robot.movel((0.26055, -0.03720, 0.23795, 2.204, 2.247, -0.067), 0.05, 0.05) 
    time.sleep(0.2)
    tool_changer.write(unlock)
    time.sleep(0.2)
    robot.set_payload(normal_payload)
    time.sleep(0.2)
    robot.movel((0.26055, -0.03720, 0.28893, 2.204, 2.247, -0.067), 0.1, 0.1)
    robot.movel((0.26055, -0.03720, 0.48702, 2.204, 2.247, -0.067), 0.3, 0.3)
    home(robot, 0.5, 0.5)

# Calculate the orientation for grinding based on the points
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

# Generate waypoints for grinding
def offset(corner, offset, normal):
   corner_new = corner - offset*normal
   return corner_new

def generateWaypoints(grid_size, lift_distance, lower, rx, ry, rz, points):
    op1, op2, op3, op4 = points
    normal_vector = normalize(np.cross(op2 - op1, op3 - op1))
    # Ensure normal vector points upwards; adjust as per your coordinate system
    if normal_vector[2] < 0:
        normal_vector = -normal_vector
    p1 = offset(op1, lower, normal_vector)
    p2 = offset(op2, lower, normal_vector)
    p3 = offset(op3, lower, normal_vector)
    p4 = offset(op4, lower, normal_vector)
    normal_vector = normalize(np.cross(p2 - p1, p3 - p1))
    if normal_vector[2] < 0:
        normal_vector = -normal_vector
    move_vector = p2 - p1
    shift_vector = normalize(p4 - p1) * grid_size
    num_passes = int(np.linalg.norm(p4 - p1) / grid_size) + 1
    waypoints = []
    current_position = p1.copy()
    for pass_num in range(num_passes):
        move_end = current_position + move_vector
        waypoints.append((current_position[0], current_position[1], current_position[2], rx, ry, rz))
        waypoints.append((move_end[0], move_end[1], move_end[2], rx, ry, rz))
        lifted_position = move_end + lift_distance * normal_vector       
        waypoints.append((lifted_position[0], lifted_position[1], lifted_position[2], rx, ry, rz))
        if pass_num < num_passes - 1:
            next_start_at_lifted_height = current_position + shift_vector + lift_distance * normal_vector
            waypoints.append((next_start_at_lifted_height[0], next_start_at_lifted_height[1], next_start_at_lifted_height[2], rx, ry, rz))
            next_start_lowered = next_start_at_lifted_height - lift_distance * normal_vector
            waypoints.append((next_start_lowered[0], next_start_lowered[1], next_start_lowered[2], rx, ry, rz))
            current_position = next_start_lowered
    return waypoints


# Perform the grinding task
def grindSurface(ur_control, acc, vel, numPasses, points, tool_changer, tool): #
    home(ur_control.robot, 0.5, 0.5)
    lock = 0
    unlock = 1
    tool_off = 0 
    tool_on = 1
    marker_payload = 1.200
    normal_payload = 1.100
    normal_tcp = (0, 0, 0, 0, 0, 0)
    marker_tcp = (0, 0, 0.180533, 0.0017, 3.1368, -0.0013)
    waypoints = []
    gridSize = 0.01
    liftDistance = 0.01
    getMarker(ur_control.robot, tool_changer, unlock, lock, marker_payload, marker_tcp)
    ur_control.robot.set_payload(marker_payload)
    ur_control.robot.set_tcp(marker_tcp)
    linearPosition = ur_control.robot.getl()
    ur_control.robot.movel(linearPosition[:3]+[0,0,0], 0.2, 0.2)
    joints = ur_control.robot.getj()
    ur_control.robot.movej((joints[0], joints[1], joints[2], joints[3], joints[4], joints[5] + np.pi), 0.4, 0.4)
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
    linearPosition = ur_control.robot.getl() 
    rx = linearPosition[3] 
    ry = linearPosition[4]
    rz = linearPosition[5]
    waypoints = []
    PASSES = 1
    count = 0
    lowerDistance = 0.005
    # tool.write(tool_on)
    while count <= numPasses - 1:
        if count == 0:
            waypoints = generateWaypoints(gridSize, liftDistance, 0 * count, rx, ry, rz, points)
        else:
            waypoints = generateWaypoints(gridSize, liftDistance, lowerDistance * count, rx, ry, rz, points)
        ur_control.show_path(waypoints) 
        passOver = 0
        while passOver <= PASSES - 1:
            for x in waypoints:
                ur_control.robot.movel(x,acc,vel)
            passOver = passOver+ 1
            print("we are on passover: ", passOver)
        count = count + 1
        print("We are on counter: ", count)
    tool.write(tool_off)
    home(ur_control.robot, 0.5, 0.5)
    returnMarker(ur_control.robot, tool_changer, unlock, normal_payload, normal_tcp)
    time.sleep(0.2)
    ur_control.robot.set_payload(normal_payload)
    time.sleep(0.1)
    ur_control.robot.set_tcp(normal_tcp)
        # Clean
    ur_control.clear_path()

# ROS2 Node for controlling the UR robot
class URControlNode(Node):
    def __init__(self):
        super().__init__('ur_control_node')
        self.subscription = self.create_subscription(PoseArray, 'repair_area/expo_marker', self.pose_array_callback, 10)
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
                robot = urx.Robot("172.16.3.131")
                time.sleep(0.3)
                connected = True
            except:
                tries += 1
                print(f"Connection attempt {tries} failed.")
                time.sleep(1)  # Wait for a second before next attempt
        if connected:
            try:
                points = np.array(self.points_list)
                self.robot = urx.Robot(self.robot_ip)
                board = Arduino('/dev/ttyACM0')
                tool_relay_pin_number = 7
                tool = board.get_pin(f'd:{tool_relay_pin_number}:o')
                tool_changer_relay_pin_number = 8
                tool_changer = board.get_pin(f'd:{tool_changer_relay_pin_number}:o')
                removemm = 1      #Note to Santi, you can add a prompt here that requests the user to input how many mm they want to grind off
                numPasses = np.floor(removemm / 0.5)
                home(self.robot,0.8,0.8)
                grindSurface(self, 0.1, 0.1, numPasses, points, tool_changer, tool) # 
                home(self.robot,0.8,0.8)
                self.robot.close()
                self.robot = None
            except Exception as e:
                print(f"Exception in control loop: {e}")

        else:
            print("Could not connect")

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