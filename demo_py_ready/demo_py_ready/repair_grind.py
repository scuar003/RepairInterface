import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import numpy as np
from pyfirmata import Arduino 
import urx


def deg2rad(deg):
    return deg * np.pi / 180


def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm


def unlockTool(relay_pin, t):
    relay_pin.write(0)  # Set the pin to HIGH (ON)
    time.sleep(t)


def lockTool(relay_pin, t):
    relay_pin.write(1)  # Set the pin to LOW (OFF)
    time.sleep(t)


def home(robot, acc, vel):
    home = (
        deg2rad(-90),
        deg2rad(-90),
        deg2rad(-90),
        deg2rad(-90),
        deg2rad(90),
        deg2rad(0),
    )
    robot.movej(home, acc, vel)


def getPCutter(robot, relay_pin):
    home(robot, 0.7, 0.7)
    robot.set_tcp((0, 0, 0, 0, 0, 0))
    startPose = (
        deg2rad(-161.57),
        deg2rad(-136.59),
        deg2rad(-57.66),
        deg2rad(-73.50),
        deg2rad(89.72),
        deg2rad(109.87),
    )
    toolPose = (-0.8353, -0.0942, 0.00913, 3.141, -0.014, -0.006)
    endPose = (-0.8353, -0.0942, 0.33297, 3.141, -0.014, -0.006)
    robot.movej(startPose, 0.8, 0.8)
    robot.movel(endPose, 0.4, 0.4)
    time.sleep(1)
    unlockTool(relay_pin, 1)
    robot.movel(toolPose, 0.1, 0.1)
    time.sleep(1)
    lockTool(relay_pin, 1)
    robot.movel(endPose, 0.3, 0.3)
    home(robot, 0.7, 0.7)


def returnPCutter(robot, relay_pin):
    home(robot, 0.7, 0.7)
    robot.set_tcp((0, 0, 0, 0, 0, 0))
    startPose = (
        deg2rad(-176.20),
        deg2rad(-137.25),
        deg2rad(-56.70),
        deg2rad(-74.05),
        deg2rad(90.21),
        deg2rad(94.97),
    )
    toolPose = (-0.8353, -0.0942, 0.00913, 3.141, -0.014, -0.006)
    endPose = (-0.8353, -0.0942, 0.33297, 3.141, -0.014, -0.006)
    robot.movej(startPose, 0.8, 0.8)
    robot.movel(endPose, 0.05, 0.5)
    robot.movel(toolPose, 0.1, 0.1)
    time.sleep(1)
    unlockTool(relay_pin, 1)
    robot.movel(endPose, 0.3, 0.3)
    home(robot, 0.7, 0.7)
    unlockTool(relay_pin, 1)


def rotation_matrix_to_rotation_vector(rot):
    theta = np.arccos((np.trace(rot) - 1) / 2)
    if theta != 0:
        rx = (rot[2, 1] - rot[1, 2]) / (2 * np.sin(theta))
        ry = (rot[0, 2] - rot[2, 0]) / (2 * np.sin(theta))
        rz = (rot[1, 0] - rot[0, 1]) / (2 * np.sin(theta))
        return np.array([-rx, -ry, rz]) * theta
    else:
        return np.array([0, 0, 0])


def findSurface():
    # p1 = np.array([-0.1653, -0.7599, 0.0611])
    # p2 = np.array([-0.1653, -0.6761, 0.0611])
    # p3 = np.array([-0.2285, -0.6761, 0.0611])
    # p4 = np.array([-0.2285, -0.7592, 0.0611])
    p1 = np.array([0.1391, -0.7978, 0.057])
    p2 = np.array([0.1391, -0.7019, 0.057])
    p3 = np.array([0.0799, -0.7019, 0.057])
    p4 = np.array([0.0799, -0.7978, 0.057])
    targetSurface = (p1, p2, p3, p4)
    return targetSurface


def getOrientation():
    points = findSurface()
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


def offset(corner, offset, normal):
    corner_new = corner - offset * normal
    return corner_new


def generateWaypoints(grid_size, lift_distance, lower, rx, ry, rz):
    points = findSurface()
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
        waypoints.append(
            (current_position[0], current_position[1], current_position[2], rx, ry, rz)
        )
        waypoints.append((move_end[0], move_end[1], move_end[2], rx, ry, rz))
        lifted_position = move_end + np.array([0, 0, lift_distance])
        waypoints.append(
            (lifted_position[0], lifted_position[1], lifted_position[2], rx, ry, rz)
        )
        if pass_num < num_passes - 1:
            next_start_at_lifted_height = (
                current_position + shift_vector + np.array([0, 0, lift_distance])
            )
            waypoints.append(
                (
                    next_start_at_lifted_height[0],
                    next_start_at_lifted_height[1],
                    next_start_at_lifted_height[2],
                    rx,
                    ry,
                    rz,
                )
            )
            waypoints.append(
                (
                    next_start_at_lifted_height[0],
                    next_start_at_lifted_height[1],
                    next_start_at_lifted_height[2],
                    rx,
                    ry,
                    rz,
                )
            )
            next_start_lowered = next_start_at_lifted_height - np.array(
                [0, 0, lift_distance]
            )
            waypoints.append(
                (
                    next_start_lowered[0],
                    next_start_lowered[1],
                    next_start_lowered[2],
                    rx,
                    ry,
                    rz,
                )
            )
            current_position = next_start_lowered
    return waypoints


def grindSurface(robot, acc, vel, numPasses, grinder):
    home(robot, 0.5, 0.5)
    # getPCutter(robot, relay_pin)
    robot.set_payload(2.300)
    robot.set_tcp((0, -0.143, 0.16169, 0, 0, 0))
    waypoints = []
    gridSize = 0.01
    liftDistance = 0.005
    # orientation = getOrientation()
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
    PASSES = 5
    GRIND_ANGLE = 0.420
    count = 0
    lowerDistance = 0.0005
    grinder.write(0)
    while count <= numPasses - 1:
        if count == 0:
            waypoints = generateWaypoints(
                gridSize, liftDistance, 0 * count, rx, ry, rz + GRIND_ANGLE
            )
        else:
            waypoints = generateWaypoints(
                gridSize, liftDistance, lowerDistance * count, rx, ry, rz + GRIND_ANGLE
            )
        passOver = 0
        while passOver <= PASSES - 1:
            for x in waypoints:
                robot.movel(x, acc, vel)
                tempx.append(x[0])
                tempy.append(x[1])
            passOver = passOver + 1
            print("we are on passover: ", passOver)
        count = count + 1
        print("We are on counter: ", count)
    grinder.write(1)
    home(robot, 0.5, 0.5)
    robot.set_tcp((0, 0, 0, 0, 0, 0))
    time.sleep(0.1)


class URControlNode(Node):
    def __init__(self):
        super().__init__("ur_grind_action")
        self.subscription = self.create_subscription(
            String, "repair_command", self.command_callback, 10
        )
        self.robot_ip = "172.16.3.114"  # Replace with your robot's IP address
        self.robot = None

    def command_callback(self, msg):
        if msg.data == "grinding":
            print("Grinding surface ")
            self.repairAction()

    def repairAction(self):
        connected = False
        tries = 0
        maxTries = 5
        while not connected and tries < maxTries:
            try:
                time.sleep(0.3)
                robot = urx.Robot("172.16.3.114")
                time.sleep(0.3)
                connected = True
            except:
                tries += 1
                print(f"Connection attempt {tries} failed.")
                time.sleep(1)  # Wait for a second before next attempt
        if connected:
            board = Arduino('/dev/ttyACM0')
            relay_pin_number = 7
            relay_pin = board.get_pin(f'd:{relay_pin_number}:o')
            relay_pin.write(1)
            removemm = 2
            numPasses = np.floor(removemm / 0.5)
            home(robot,0.8,0.8)
            grindSurface(robot, 0.004, 0.004, numPasses, relay_pin)
            home(robot,0.8,0.8)
            robot.close()


def main():
    rclpy.init()
    ur_control_node = URControlNode()
    rclpy.spin(ur_control_node)
    ur_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
