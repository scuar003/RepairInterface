import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import numpy as np
import urx
from pyfirmata import Arduino


def in2m(inches):
    return inches / 39.3701


def deg2rad(deg):
    return deg * np.pi / 180


def cosd(a):
    return np.cos(deg2rad(a))


def sind(a):
    return np.sin(deg2rad(a))


def home(robot):
    robot.movej(
        (
            deg2rad(-90.00),
            deg2rad(-90),
            deg2rad(-90),
            deg2rad(-90),
            deg2rad(90),
            deg2rad(0),
        ),
        0.8,
        0.80,
    )


class URControlNode(Node):
    def __init__(self):
        super().__init__("ur_pipe_action")
        self.subscription = self.create_subscription(
            String, "repair_command", self.command_callback, 10
        )
        self.robot_ip = "172.16.3.114"  # Replace with your robot's IP address
        self.robot = None

    def command_callback(self, msg):
        if msg.data == "cut pipe":
            print("Cutting Pipe ")
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
            board = Arduino("/dev/ttyACM0")
            relay_pin_number = 7
            relay_pin = board.get_pin(f"d:{relay_pin_number}:o")
            relay_pin.write(1)
            robot.set_tcp((0, 0, 0, 0, 0, 0))
            time.sleep(0.2)

            home(robot)
            robot.movej(
                (
                    deg2rad(-34.17),
                    deg2rad(-105.55),
                    deg2rad(-111.88),
                    deg2rad(-53.43),
                    deg2rad(90.25),
                    deg2rad(177.22),
                ),
                0.6,
                0.6,
            )
            relay_pin.write(0)
            a = 0.004
            v = a
            p1 = [0.340, -0.514, 0.180, 3.15, 0, 0.01]
            p2 = [0.314, -0.514, 0.180, 3.15, 0, 0.01]
            p3 = [0.292, -0.466, 0.180, 3.15, 0, 0.01]
            p4 = [0.266, -0.445, 0.180, 3.15, 0, 0.01]
            p5 = [0.236, -0.433, 0.180, 3.15, 0, 0.01]
            p6 = [0.195, -0.433, 0.180, 3.15, 0, 0.01]
            p7 = [0.154, -0.455, 0.180, 3.15, 0, 0.01]
            p8 = [0.135, -0.505, 0.180, 3.15, 0, 0.01]
            p9 = [0.140, -0.565, 0.180, 3.15, 0, 0.01]
            p10 = [0.158, -0.591, 0.180, 3.15, 0, 0.01]
            p11 = [0.185, -0.602, 0.180, 3.15, 0, 0.01]
            p12 = [0.220, -0.611, 0.180, 3.15, 0, 0.01]
            p13 = [0.258, -0.597, 0.180, 3.15, 0, 0.01]
            p14 = [0.288, -0.554, 0.180, 3.15, 0, 0.01]
            p15 = [0.304, -0.557, 0.180, 3.15, 0, 0.01]
            p16 = [0.308, -0.527, 0.180, 3.15, 0, 0.01]
            p17 = [0.308, -0.504, 0.180, 3.15, 0, 0.01]
            robot.movel(p1, 0.01, 0.01)
            robot.movel(p2, a, a)
            robot.movels(
                (p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15, p16, p17),
                a,
                a,
            )
            relay_pin.write(1)
            home(robot)
            robot.close()
        else:
            print("Connection failed, check robot status.")


def main():
    rclpy.init()
    ur_control_node = URControlNode()
    rclpy.spin(ur_control_node)
    ur_control_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
