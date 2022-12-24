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
from pathlib import Path

# Name of this ROS package.
ROSAPP='jetbot_move'

# Topics to subscribe to
DANCE_ROUTINE_DEMO='/dance/demo'
DANCE_ROUTINE_JSON='/dance/raw'
DANCE_ROUTINE_START='/dance/start'

class Dance(Node):
    '''
    This class will move the JetBot robot according to a defined dance routine. 
    It subscribes to the following topics:

    /dance/demo -- String (demo name) -- Will pull the routine from /src/jetbot_move/routines folder in this ROS package.
    /dance/start -- String (start) -- Will start a routine at a defined local file path DANCE_ROUTINE_PATH.
    /dance/raw -- String (JSON) -- A raw JSON string containing the routine.
    
    Examples:
    ros2 topic pub /dance/demo std_msgs/String "data: autumn" -- This command will use the routine: {ROS_PACKAGE_SHARE}/routines/autumn.json at timestamp 1607551421.
    ros2 topic pub /dance/start std_msgs/String "data: start" -- This command will use the routine defined in DANCE_ROUTINE environment variable.
    ros2 topic pub /dance/raw std_msgs/String "data: { <DANCE_JSON> }" -- This command will use the raw JSON in the message and start the dance imminently. 

    Routine JSON is the following structure:
    {
        "name": "<DANCE_ROUTINE_NAME>",
        "songName": "<SONG_NAME>",
        "artist": "<ARTIST_NAME>",
        "audioURL": "<URL_TO_AUDIO_FILE>",
        "dancers": {
            "<ROBOT_DANCER_POSITION>": {
                "startPosition": "<START_ROBOT_POSITION>",
                "routine": {
                    "<TIME_STEP_TO_SEND_MOVE>": "<MOVE_COMMAND>",
                }
            }
        }
    }
    '''
    
    def __init__(self):
        super().__init__('dance')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('cmd_vel_topic', 'cmd_vel'),
                ('cmd_str_topic', 'cmd_str'),
                ('dance_start_demo', 'dance/demo'),
                ('dance_start_json', 'dance/raw'),
                ('dance_start_routine', 'dance/start'),
                ('loaded_routine', '/home/ubuntu/routines/autumn.json'),
                ('home_routines', '/home/ubuntu/routines'),
                ('auto_start', 'False'),
                ('speed', 3)
            ]
        )
        
        # Set constants, setup publishers and subscribe to topics.
        self.get_logger().info("Initializing dance settings...")
        self.get_logger().info("Command topics:")
        self.get_logger().info(" -- Velocity: %s/%s" % (self.get_namespace(), self.get_parameter('cmd_vel_topic')._value))
        self.get_logger().info(" -- String: %s/%s" % (self.get_namespace(),self.get_parameter('cmd_str_topic')._value))

        self.dancer_position = 'lead'
        self.twist = Twist()
        self.rate = self.create_rate(self.get_parameter('speed')._value, self.get_clock())
        self.routine = []
        self.step_counter = 0
        self.dancing = False
        self.start_timestamp = None
        
        self._cmd_vel_pub = self.create_publisher(Twist, self.get_parameter('cmd_vel_topic')._value, 1)
        self._cmd_str_pub = self.create_publisher(String, self.get_parameter('cmd_str_topic')._value, 1)
        self._dance_start_pub = self.create_publisher(String, self.get_parameter('dance_start_routine')._value, 1)   
        self.create_subscription(String, self.get_parameter('dance_start_demo')._value, self.start_demo, 1)
        self.create_subscription(String, self.get_parameter('dance_start_json')._value, self.start_json, 1)
        self.create_subscription(String, self.get_parameter('dance_start_routine')._value, self.start_routine, 1)
        
        msg = String()
        msg.data = self.get_parameter('auto_start')._value
        self._dance_start_pub.publish(msg);
    
    def set_routine_array(self, routine):
        # Convert easy JSON document structure into more usable ordered array.
        self.routine = []
        for key, value in routine.items():
            self.routine.append({"step": int(key), "value": value})
    
    def set_routine(self, routine_path):
        if (os.path.exists(routine_path)):
            with open(routine_path) as f:
                routine_json = json.load(f) 
            self.get_logger().info("Routines path: %s" % routine_path)
            self.set_routine_array(routine_json['dancers'][self.dancer_position]['routine'])
            return True
        else:
            self.get_logger().error("Routine '%s' could not be found. " % routine_path)
            return False
            
    def start_demo(self, data):
        # Start a new demo dance based on the preloaded local demo routines in /src/jetbot_move/routines.
        if (not self.dancing):
            routine = data.data
            self.get_logger().info('Dance demo message recieved. Looking up routine: %s' % routine)
            routine_path = "%s/demo_routines/%s.json" % (get_package_share_directory('jetbot_move'), routine)
            if (self.set_routine(routine_path)):
                self.start_timestamp = int(time.time())
                self.dancing = True
                self.dance(0)
            else:
                self.get_logger().error("Dance not started. Please check the routine!")
        else:
            self.get_logger().info('Already dancing!')
            
    def start_routine(self, start_routine_name):
        # Start a new dance routine based on a routine locally deployed with Greengrass.
        try: 
            if (start_routine_name.data and not self.dancing):
                if Path(self.get_parameter('home_routines')._value+"/"+start_routine_name.data).is_file():
                    routine_path = self.get_parameter('home_routines')._value+"/"+start_routine_name.data
                else:
                    routine_path = self.get_parameter('loaded_routine')._value
                    
                self.get_logger().info('Dance demo message recieved. Looking up routine: %s' % start_routine_name.data)
                self.get_logger().info("Routines path: %s" % routine_path)
                if (self.set_routine(routine_path)):
                    self.start_timestamp = int(time.time())
                    self.dancing = True
                    self.dance(0)
                else:
                    self.get_logger().error("Dance not started. Please check the routine!")
            else:
                self.get_logger().info('Already dancing!')
        except:
            self.get_logger().error('Could not start dance! Please check the routine!')

    def start_json(self, data):
        # Recieve raw JSON from topic /dance/raw and start dance.
        if (not self.dancing):
            self.get_logger().info('JSON Routine recieved: %s' % data.data)
            routine_json = json.loads(data.data)
            self.set_routine_array(routine_json['dancers'][self.dancer_position]['routine'])
            self.start_timestamp = int(time.time())
            self.dancing = True
            self.dance(0)
        else:
            self.get_logger().info('Already dancing!')
            
    def move(self, value):
        # Publish move commands to the move node. If it is a Twist message, use cmd_vel.
        twist = value.split(" ")
        if len(twist) > 2:
            self.get_logger().info('Robot %s: New twist message %s.' % (self.dancer_position, value))
            self.twist.linear.x = float(twist[0])
            self.twist.linear.y = float(twist[1])
            self.twist.linear.z = float(twist[2])
            self.twist.angular.x = float(twist[3])
            self.twist.angular.y = float(twist[4])
            self.twist.angular.z = float(twist[5])
            self._cmd_vel_pub.publish(self.twist)
            return
        else:
            self.get_logger().info('Robot %s: New move %s.' % (self.dancer_position, value))
            msg = String()
            msg.data = value
            self._cmd_str_pub.publish(msg)
            return
          
    def dance(self, step): 
        # Recursive Dance Step Function, iterates by time step (controlled by SPEED in Hz), then matches current timestep to dance step in the routine.
        # Example: 
        # time step (20) : dance step [3] > { step: 20, value: "left" }, then move left and increment dance step
        # time step (21) : dance step [4] > { step: 23, value: "right"}, NO MATCH : NO CHANGE
        # time step (22) : dance step [4] > { step: 23, value: "right"}, NO MATCH : NO CHANGE
        # time step (23) : dance step [4] > { step: 23, value: "right"}, then move right and increment dance step
        # time step (24) : dance step [5] > { step: 32, value: "forward"}, NO MATCH : NO CHANGE
        next_move = "NO CHANGE"
        self.get_logger().info('Next move started...')
        self.get_logger().info('Length: %i' % len(self.routine))
        if (int(time.time())>=self.start_timestamp):
            if (self.routine[self.step_counter]['step'] == step):
                next_move = self.routine[self.step_counter]['value']
                if (next_move == 'end'):
                    self.dancing = False
                else:
                    self.move(next_move)
                self.step_counter += 1

            self.get_logger().info('Robot time step %i, dance step %i: %s.' % (step, self.step_counter, next_move))
            if (self.dancing and self.step_counter < len(self.routine)):
                self.get_logger().info('Dancing: %s' % self.dancing)
                time.sleep(0.5)
                self.get_logger().info('Dancing continues: %s' % self.dancing)
                next_step = step+1
                self.get_logger().info('Next step %i' % next_step)
                self.dance(next_step)
            else:
                self.get_logger().info('Dance finished!')
                self.move("stop")
                self.step_counter = 0
        else:
            time_left = int(self.start_timestamp-time.time())
            self.get_logger().info('Dance routine invoked. Starting in %i seconds' % time_left)
            self.rate.sleep()
            self.dance(step)
            

def main(args=None):

    rclpy.init(args=args)
    node = Dance()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    
if __name__ == '__main__':
        main()