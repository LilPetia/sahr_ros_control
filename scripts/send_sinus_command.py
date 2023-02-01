#!/usr/bin python3

import rclpy
from rclpy.node import Node
import math

from bitbots_msgs.msg import JointCommand


DYNAMIXEL_CMD_TOPIC = "/DynamixelController/command"
JOINT_NAME = "RShoulderPitch"
PUBLISH_RATE = 1000

# sin function
FREQUENCY = 0.5
AMPLITUDE = 72 #degree

if __name__ == "__main__":
    msg = JointCommand(
        joint_names=[JOINT_NAME],
        velocities=[-1],
        accelerations=[-1],
        max_currents=[-1])

    rclpy.init(args=None)
    node = Node("Sinus_Publisher")
    pub = node.create_publisher(JointCommand, DYNAMIXEL_CMD_TOPIC, 1)

    rate = node.create_rate(PUBLISH_RATE)
    i = 1
    while rclpy.ok():
        time = node.get_clock().now()
        i += 0.1
        position = math.radians(AMPLITUDE) * math.sin(2 * math.pi * FREQUENCY * i)
        print("send message wih position{}".format(position))
        # msg.header.stamp = time
        msg.positions=[position]
        pub.publish(msg)
        #rate.sleep(1)
        import time
        time.sleep(1)