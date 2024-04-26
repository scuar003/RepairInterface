import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import urx
from pyfirmata import Arduino

import numpy as np
import time 

def deg2rad(deg):
    return deg * np.pi / 180

#home
def home(robot, acc, vel):
    home_position = (deg2rad(-90), deg2rad(-90), deg2rad(-90), deg2rad(-90), deg2rad(90), deg2rad(0))
    robot.movej(home_position, acc, vel)

#Grinder
def getGrinder(robot, acc, vel, lock, unlock, tool_changer):
    pass
    

def returnGrinder():
    pass

#Vacuum
def getVacuum(robot, acc, vel, lock, unlock, tool_changer):
    home(robot, acc, vel)
    robot.set_tcp((0,0,0,0,0,0))
    vacuum_tcp =()
    vacuum_payload = ()
    vacuum_cog = ()
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

def returnVacuum():
    pass

#expo_marker
def getExpoMarker():
    pass

def returnExpoMarker():
    pass

def toolChanger():
    board = Arduino('/dev/ttyACM1')
    tool_changer_relay_pin_number = 8
    tool_changer = board.get_pin(f'd:{tool_changer_relay_pin_number}:o')
    return tool_changer

def toolActuator():
    board = Arduino('/dev/ttyACM1')
    tool_relay_pin_number = 7
    tool = board.get_pin(f'd:{tool_relay_pin_number}:o')
    return tool 

def getTool(msg_tool, robot):
    acc, vel = 0.5, 0.5
    lock, unlock = 0, 1
    tool_changer = toolChanger()
    if msg_tool.data == "grinder": getGrinder(robot, acc, vel, lock, unlock, tool_changer)
    if msg_tool.data == "vacuum" : getVacuum(robot, acc, vel)
    if msg_tool.data == "expo_marker" : getExpoMarker(robot, acc, vel)

def returnTool(current_tool):
    if current_tool == 'grinder': returnGrinder()
    if current_tool == 'vacuum': returnVacuum()
    if current_tool == 'expo_marker': returnExpoMarker()


class GetReturnTools(Node):
    def __init__(self):
        super().__init__('Tool Interaction')
        self.subscription = self.create_subscription(String, 'menu_action', self.menuActionCallback, 10)
        #make current tool a subscriber 
        self.current_tool = ''
        self.robot_ip = '172.16.0.4'

    def menuActionCallback(self, msg):
        self.connectToRobot()
        if self.current_tool != '': returnTool(self.current_tool)
        else: getTool(msg, self.robot)
        
    def connectToRobot(self):
        connected = False
        tries = 0
        maxTries = 5
        while not connected and tries < maxTries:
            try:
                time.sleep(0.3)
                self.robot = urx.Robot(self.robot_ip)
                time.sleep(0.3)
                connected = True
            except:
                tries +1
        if connected:
            self.robot = urx.Robot(self.robot_ip)
            return self.robot
        else:
            print("Can't connect to the Robot")

def main(args = None ):
    rclpy.init(args=args)
    tools_node = GetReturnTools()
    rclpy.spin(tools_node)
    tools_node.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()
        
    

