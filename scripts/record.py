#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/rae/right/image_raw',  # Updated topic
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.cv_image = None  # Initialize cv_image
        self.record_video = True  # Start recording immediately
        self.video_writer = None
        self.frames_to_record = 300  # 30 fps * 3 seconds

    def listener_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV format
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            if self.record_video:
                if self.video_writer is None:
                    frame_height, frame_width, _ = self.cv_image.shape
                    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # MP4 format
                    self.video_writer = cv2.VideoWriter('recorded_video.mp4', fourcc, 30, (frame_width, frame_height))
                    self.get_logger().info('Video recording started.')  # Log message when recording starts

                self.video_writer.write(self.cv_image)

                self.frames_to_record -= 1
                if self.frames_to_record <= 0:
                    self.video_writer.release()
                    self.get_logger().info('Video recording complete.')
                    self.record_video = False
        except CvBridgeError as e:
            self.get_logger().error('CvBridge Error: {}'.format(e))

            
def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'An error occurred: {e}')

    # Ensure resources are released when shutting down
    if image_subscriber.video_writer:
        image_subscriber.video_writer.release()

    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
