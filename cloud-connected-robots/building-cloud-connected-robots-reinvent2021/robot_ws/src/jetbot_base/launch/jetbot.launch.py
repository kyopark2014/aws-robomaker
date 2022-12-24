#!/usr/bin/env python
"""
Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software
is furnished to do so.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED,INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import os

DEFAULT_SPEED = os.environ.get('DEFAULT_SPEED', '0.2')
MAX_PWM =  os.environ.get('MAX_PWM', '115')
I2C_BUS = os.environ.get('I2C_BUS', '1')
ROBOT_ID = os.environ.get('ROBOT_ID', 'jetbot')
CAMERA = os.environ.get('CAMERA', 'false')
LIDAR = os.environ.get('LIDAR', 'true')
WLAN0_IP = os.environ.get('WLAN0_IP', 'No WLAN0 IP')
ETH0_IP = os.environ.get('ETH0_IP', 'No ETH0 IP')

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    launch_description = [

        Node(
             package='jetbot_base', executable='jetbot_motor_controller', output='screen',
             parameters=[
                {"default_speed": DEFAULT_SPEED},
                {"i2c_bus": I2C_BUS},
                {"max_pwm": MAX_PWM}
             ],
             namespace=ROBOT_ID,
             name=['jetbot_motor_controller'])

#        Node(
#             package='jetbot_base', executable='jetbot_oled_display', output='screen',
#             parameters=[
#                {"eth0_ip": ETH0_IP},
#                {"wlan0_ip": WLAN0_IP}
#             ],
#             namespace=ROBOT_ID,
#             name=['jetbot_oled_display']),

     ]

#    if (CAMERA=='true'):
#        launch_description.append(Node(
#             package='jetbot_base', executable='jetbot_camera', output='screen',
#             namespace=ROBOT_ID,
#             name=['jetbot_camera']))

    return LaunchDescription(launch_description)

if __name__ == '__main__':
    generate_launch_description()