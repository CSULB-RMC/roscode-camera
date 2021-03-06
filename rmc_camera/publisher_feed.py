import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Int8

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

        self.capture.set(3, 256)
        self.capture.set(4, 144)

        self.bridge = CvBridge()

        self.publisher_ = self.create_publisher(Image, "camera_feed", 10)
        self.cameraid_subscriber = self.create_subscription(Int8, "camera_id", self.camera_setup, 10)
        self.timer = self.create_timer(self.TIMER_PERIOD, self.timer_callback)

        self.cameraid_subscriber

    def camera_setup(self, msg):
        self.VIDEO_FEED = "/dev/video" + str(msg.data)
        self.capture = cv.VideoCapture(self.VIDEO_FEED, self.VIDEO_DRIVER)

        self.capture.set(3, 256)
        self.capture.set(4, 144)
        

    def timer_callback(self):
        """
        timer call back function for periodically publishing camera feed data
        """

        ret, frame = self.capture.read()

        if not ret:
            self.get_logger().error("camera not initialized")

        if type(frame) is np.ndarray:
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
