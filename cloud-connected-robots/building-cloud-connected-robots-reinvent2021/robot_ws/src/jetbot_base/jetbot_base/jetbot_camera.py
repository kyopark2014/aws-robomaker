#!/usr/bin/python3

import logging
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
import time
import signal
import json

class JetbotCamera(Node):
    
    def __init__(self):
        super().__init__('jetbot_camera')
        self.pub = self.create_publisher(Image, 'jetbot_camera/raw', 10)
        timer_period = 1.0
        pipeline = self.gstreamer_pipeline(flip_method=0)
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if self.cap.isOpened():
            self.tmr = self.create_timer(timer_period, self.timer_callback)
        else:
            self.get_logger().error('Could not open camera capture...')
            
    def timer_callback(self):
        ret_val, img = self.cap.read()
        resize_image = cv2.resize(img, (224, 224))
        msg_frame = CvBridge().cv2_to_imgmsg(resize_image, 'bgr8')
        self.get_logger().info('Publishing image...')
        self.pub.publish(msg_frame)
    
    def release_cap(self):
        self.cap.release()
        cv2.destroyAllWindows()
        
    def gstreamer_pipeline(
        self,
        capture_width=1280,
        capture_height=720,
        display_width=1280,
        display_height=720,
        framerate=60,
        flip_method=0
    ):
        return (
            'nvarguscamerasrc ! '
            'video/x-raw(memory:NVMM), '
            'width=(int)%d, height=(int)%d, '
            'format=(string)NV12, framerate=(fraction)%d/1 ! '
            'nvvidconv flip-method=%d ! '
            'video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! '
            'videoconvert ! '
            'video/x-raw, format=(string)BGR ! appsink' %
            (capture_width, capture_height, framerate, flip_method, display_width, display_height)
        )

def main(args=None):
    rclpy.init(args=args)

    node = JetbotCamera()

    def stop_node(*args):
        node.release_cap()
        print("Releasing capture stopping the node...")
        rclpy.shutdown()
        return True
    
    signal.signal(signal.SIGINT, stop_node)
    signal.signal(signal.SIGTERM, stop_node)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.release_cap()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()