import rclpy
from rclpy.node import Node
import urx


import numpy as np
import time 
from std_msgs.msg import String


def deg2rad(deg):
    return deg * np.pi / 180

def home(robot, acc, vel):
    home_position = (deg2rad(-90), deg2rad(-90), deg2rad(-90), deg2rad(-90), deg2rad(90), deg2rad(0))
    camera_position =(deg2rad(-90.51), deg2rad(-53.88), deg2rad(-76.40), deg2rad(-95.10), deg2rad(88.94), deg2rad(0))
    robot.movej(home_position, acc, vel)
    robot.movej(camera_position, acc, vel)

class URMoveHome(Node):
    def __init__(self):
        super().__init__('move_home_node')
        self.subscription = self.create_subscription(String, 'menu_action', self.menuActionCallback, 10)
        self.robot_ip = "172.16.0.11"

    def menuActionCallback(self, msg):
        if msg.data == "move home":
            print('Moving Home')
            self.moveHome()

    def moveHome(self):
        connected = False
        tries = 0
        maxTries = 5
        while not connected and tries < maxTries:
            try:
                time.sleep(0.3)
                robot = urx.Robot(self.robot_ip)
                time.sleep(0.3)
                connected = True
            except:
                tries += 1
                print(f"Connection Failed after {tries}.")
                time.sleep(1)
        if connected: 
            self.robot = urx.Robot(self.robot_ip)
            home(self.robot, 0.8, 0.8)
        else: 
            print("Could not complete task")

def main(args = None ):
    rclpy.init(args=args)
    move_home_node = URMoveHome()
    rclpy.spin(move_home_node)
    move_home_node.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()
        

        

