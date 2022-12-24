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
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
import Adafruit_SSD1306

class OLED_Display(Node):
    
    def __init__(self):
        super().__init__('oled_display')
        # Set constants, setup publishers and subscribe to topics.
        self.get_logger().info("Initializing OLED Display...")
        self.declare_parameter('eth0_ip')
        self.declare_parameter('wlan0_ip')
        
        self.stats = {
            "MemoryUsage": "",
            "DiskUsage": ""
        }
        self.ip_address = {
            'eth0': self.get_parameter('eth0_ip')._value,
            'wlan0': self.get_parameter('wlan0_ip')._value
        }
        self.text_to_display = None

        self.start_subscriptions()
        self.init_display()
        
    def start_subscriptions(self):
        self.create_subscription(String, '/stats/mem_usage', self.on_memory_usage, 1)
        self.create_subscription(String, '/stats/disk_usage', self.on_disk_usage, 1)
        self.create_subscription(String, '/oled/display', self.on_user_text, 1)
    
    def on_disk_usage(self, msg):
        self.stats['DiskUsage'] = msg.data

    def on_memory_usage(self, msg):
        self.stats['MemoryUsage'] = msg.data

    def init_display(self):
        
        self.disp = Adafruit_SSD1306.SSD1306_128_32(rst=None, i2c_bus=1, gpio=1) # setting gpio to 1 is hack to avoid platform detection
        self.disp.begin()
        self.disp.clear()
        self.disp.display()
        
        # Create blank image for drawing.
        # Make sure to create image with mode '1' for 1-bit color.
        self.image = Image.new('1', (self.disp.width, self.disp.height))

        # Get drawing object to draw on image.
        self.draw = ImageDraw.Draw(self.image)

        # Draw a black filled box to clear the image.
        self.draw.rectangle((0,0,self.disp.width,self.disp.height), outline=0, fill=0)

        # Load default font.
        self.font = ImageFont.load_default()
        timer_period = 1.0
        self.tmr = self.create_timer(timer_period, self.loop_callback)
        
    def loop_callback(self):
        # First define some constants to allow easy resizing of shapes.
        padding = -2
        top = padding
        bottom = self.disp.height-padding
        # Move left to right keeping track of the current x position for drawing shapes.
        x = 0
        
        # Draw a black filled box to clear the image.
        self.draw.rectangle((0,0,self.disp.width,self.disp.height), outline=0, fill=0)

        # Write two lines of text.
        if not self.text_to_display is None:
            self.draw.text((x, top), self.text_to_display,  font=self.font, fill=255)
        else:
            self.draw.text((x, top), "eth0: %s" % str(self.ip_address['eth0']), font=self.font, fill=255)

        self.draw.text((x, top+8),  "wlan0: %s" % str(self.ip_address['wlan0']), font=self.font, fill=255)
        self.draw.text((x, top+16), str(self.stats['MemoryUsage']),  font=self.font, fill=255)
        self.draw.text((x, top+25), str(self.stats['DiskUsage']),  font=self.font, fill=255)

        self.disp.image(self.image)
        self.disp.display()
    
    def on_user_text(self, msg):
        self.text_to_display = msg.data
           

def main(args=None):

    rclpy.init(args=args)
    
    node = OLED_Display()
        
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
        
if __name__ == '__main__':
        main()