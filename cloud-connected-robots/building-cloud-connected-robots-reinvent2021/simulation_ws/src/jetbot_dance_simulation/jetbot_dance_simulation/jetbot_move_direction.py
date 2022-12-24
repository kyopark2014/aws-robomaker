#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json
import csv
import os
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time
import urllib
from ament_index_python.packages import get_package_share_directory

class JetbotMoveDirection(Node):
    
    def __init__(self):
        super().__init__('cmd_move_direction')
        
        # Set constants, setup publishers and subscribe to topics.
        self.get_logger().info("Initializing command move direction...")
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)        
        self.create_subscription(String, 'cmd_str', self.move_dir, 1)
    
    def move_dir(self, val):
        # Publish move commands to the move node. If it is a Twist message, use cmd_vel.
        move_val = val.data
        print("Message recieved: %s" % move_val)
        move_cmd = Twist()
        if move_val == "left":
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.2
        elif move_val == "right":
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = -0.2
        elif move_val == "backward":
            move_cmd.linear.x = -0.2
            move_cmd.angular.z = 0.0
        elif move_val == "forward": 
            move_cmd.linear.x = 0.2
            move_cmd.angular.z = 0.0
        elif move_val == "stop":
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
        else:
            self.get_logger().error('Direction not supported.')
        
        self._cmd_vel_pub.publish(move_cmd)
        return

def main(args=None):

    rclpy.init(args=args)

    node = JetbotMoveDirection()
        
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.move_dir("stop")
        pass

    node.destroy_node()
    
if __name__ == '__main__':
        main()