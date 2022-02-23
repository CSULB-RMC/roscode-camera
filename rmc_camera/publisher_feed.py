import rclpy
from rclpy.node import Node

# using bytes
# from std_msgs.msg import Byte
from sensor_msgs.msg import Image

import numpy as np
import cv2 as cv

from cv_bridge import CvBridge


class FeedPublisher(Node):
    TIMER_PERIOD = 0.1
    VIDEO_FEED = "/dev/video0"
    VIDEO_DRIVER = cv.CAP_V4L2

    def __init__(self):
        super().__init__("publisher_feed")
        # opencv camera capture
        self.capture = cv.VideoCapture(self.VIDEO_FEED, self.VIDEO_DRIVER)
        self.bridge = CvBridge()

        # using bytes
        # self.publisher_ = self.create_publisher(Byte, 'camera_feed', 10)
        self.publisher_ = self.create_publisher(Image, "camera_feed", 10)
        self.timer = self.create_timer(self.TIMER_PERIOD, self.timer_callback)

    def timer_callback(self):
        """
        timer call back function for periodically publishing camera feed data
        """

        ret, frame = self.capture.read()

        if not ret:
            self.get_logger().error("camera not initialized")

        # gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        # using bytes
        # self.publisher_.publish(gray.tobytes())
        self.publisher_.publish(self.bridge.cv2_to_imgmsg(frame))
        self.get_logger().info("sent camera feed")


def main(args=None):
    rclpy.init(args=args)
    publisher = FeedPublisher()
    rclpy.spin(publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    feed_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
