import rclpy
from rclpy.node import Node
import time
import signal
import sys
import json
from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class JetBotMotorController(Node):

    def __init__(self):
        super().__init__('jetbot_motor_controller')
        self.declare_parameter('i2c_bus')
        self.declare_parameter('default_speed')
        self.declare_parameter('max_pwm')
        self.declare_parameter('type', 'waveshare')
        self.start_subscriptions();
        self.set_speed(self.get_parameter('default_speed')._value)
        self.motor_driver = Adafruit_MotorHAT(i2c_bus=int(self.get_parameter('i2c_bus')._value))
        self.motor_left_ID = 1
        self.motor_right_ID = 2
        self.motor_left = self.motor_driver.getMotor(self.motor_left_ID)
        self.motor_right = self.motor_driver.getMotor(self.motor_right_ID)
        self.all_stop()
    
    def start_subscriptions(self):
        self.cmd_vel = self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 1)
        self.cmd_dir = self.create_subscription(String, 'cmd_dir', self.on_cmd_dir, 1)
        self.cmd_raw = self.create_subscription(String, 'cmd_raw', self.on_cmd_raw, 10)
        self.cmd_str = self.create_subscription(String, 'cmd_str', self.on_cmd_str, 10)
        
    # directional commands (degree, speed)
    def on_cmd_dir(self, msg):
        self.get_logger().info('cmd_dir=%s' % msg.data)
    
    # velocity, twist commands (Twist)
    def on_cmd_vel(self, msg):
        x = msg.linear.x
        y = msg.angular.z/10
		 
        if x>0 and y<0: #backward right
            self.set_pwm(self.motor_left_ID, (abs(y)+0.1))
            self.set_pwm(self.motor_right_ID, (0.2+y+0.1))
        elif x>0 and y>0: #backward left
            self.set_pwm(self.motor_left_ID, (0.2-y+0.1))
            self.set_pwm(self.motor_right_ID, (y+0.1))
        elif x<0 and y>0: #forward left
            self.set_pwm(self.motor_left_ID, (-(0.2-y)-0.1))
            self.set_pwm(self.motor_right_ID, -(y+0.1))
        elif x<0 and y<0: #forward right
            self.set_pwm(self.motor_left_ID, y-0.1)
            self.set_pwm(self.motor_right_ID, (-(0.2+y)-0.1))
        else:
            self.all_stop()
        
    # raw L/R motor commands (speed, speed)
    def on_cmd_raw(self, msg):
        self.get_logger().info(' cmd_raw=%s' % msg.data)
        move_data_recv = json.loads(msg.data)
        self.set_pwm(self.motor_left_ID, float(move_data_recv['left']))
        self.set_pwm(self.motor_right_ID, float(move_data_recv['right']))
    
    # simple string commands (left/right/forward/backward/stop)
    def on_cmd_str(self, msg):
        self.get_logger().info(' cmd_str=%s' % msg.data)
        self.move_dir(msg.data.lower())
        
    def set_pwm(self, motor_ID, value):
        
        max_pwm = float(self.get_parameter('max_pwm')._value)
        speed = int(min(max(abs(value * max_pwm), 0), max_pwm))
        
        if motor_ID == 1:
            motor = self.motor_left
        elif motor_ID == 2:
            motor = self.motor_right
        else:
            self.get_logger().error('set_pwm(%d, %f) -> invalid motor_ID=%d', motor_ID, value, motor_ID)
            return
            
        motor.setSpeed(speed)
        
        if value > 0:
            motor.run(Adafruit_MotorHAT.FORWARD)
        else:
            motor.run(Adafruit_MotorHAT.BACKWARD)
        
    def set_speed(self, speed):
        self.speed = float(speed)

    def move_dir(self, val):
        if val == "left":
            self.set_pwm(self.motor_left_ID,  self.speed)
            self.set_pwm(self.motor_right_ID, (-1 * self.speed)) 
        elif val == "right":
            self.set_pwm(self.motor_left_ID,   self.speed)
            self.set_pwm(self.motor_right_ID, self.speed) 
        elif val == "backward":
            self.set_pwm(self.motor_left_ID, self.speed)
            self.set_pwm(self.motor_right_ID, self.speed)
        elif val == "forward":
            self.set_pwm(self.motor_left_ID,  (-1 * self.speed))
            self.set_pwm(self.motor_right_ID, (-1 * self.speed))  
        elif val == "stop":
            self.all_stop()
        else:
            self.get_logger().error('Direction not supported.')

    def all_stop(self):
        self.motor_left.setSpeed(0)
        self.motor_right.setSpeed(0)
        self.motor_left.run(Adafruit_MotorHAT.RELEASE)
        self.motor_right.run(Adafruit_MotorHAT.RELEASE)

def main(args=None):

    rclpy.init(args=args)

    node = JetBotMotorController()
        
    def stop_node(*args):
        node.all_stop()
        print("Releasing i2c motors and stopping the node...")
        rclpy.shutdown()
        return True
    
    signal.signal(signal.SIGINT, stop_node)
    signal.signal(signal.SIGTERM, stop_node)
        
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
        
if __name__ == '__main__':
        main()
    