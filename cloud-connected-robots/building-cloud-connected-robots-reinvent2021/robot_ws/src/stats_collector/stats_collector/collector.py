#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json
import csv
import os
import sys
from std_msgs.msg import String
import time
import urllib
import subprocess

class StatsCollector(Node):
    
    def __init__(self):    
        super().__init__('stats_collector')
        self.get_logger().info("Initializing Stats Collection...")
        self.commands = {
            "disk_usage": "df -h | awk '$NF==\"/\"{printf \"Disk: %d/%dGB %s\", $3,$2,$5}'",
            "mem_usage": "free -m | awk 'NR==2{printf \"Mem: %s/%sMB %.2f%%\", $3,$2,$3*100/$2 }'"
        }
        timer_period = 1.0
        self.create_publishers()
        self.tmr = self.create_timer(timer_period, self.publish)
    
    def create_publishers(self):
        self.stats_publishers = {}
        for key, value in self.commands.items():
            self.stats_publishers[key] = self.create_publisher(String, 'stats/%s' % key, 10)
    
    def publish(self):
        for key, value in self.commands.items():
            _current_value = subprocess.check_output(value, shell = True )
            self.get_logger().debug('Publishing: "{0}"'.format(_current_value))
            msg = String()
            msg.data = str(_current_value.decode("utf-8"))
            self.stats_publishers[key].publish(msg)


def main(args=None):

    rclpy.init(args=args)
    
    node = StatsCollector()
        
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
        
if __name__ == '__main__':
        main()