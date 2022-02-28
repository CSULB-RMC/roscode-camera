import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge


class ObstructionPublisher(Node):
    """Obstruction publisher node to detect a drivable path
    as an array of pixel coordinates relative to the camera feed
    """

    def __init__(self):
        super().__init__("publisher_obstruction")
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image, "camera_feed", self.listener_callback, 10
        )

        self.publisher_ = self.create_publisher(
            Int16MultiArray, "publisher_obstruction", 10
        )

    def listener_callback(self, msg: Image):
        """Listen for camera feed and publish frame obstruction data.
        The Data is published a 16-byte integer array. To rebuild the data
        on the receiving node, the array should be split into an array
        of 2x2 tuples. Ex: [(x,y), (x,y) ... ]

        :param msg the message received from the camera feed
        """

        pixel_steps = 20

        # values for edge detection
        edge_detection_min_threshold = 0
        edge_detection_max_threshold = 300

        # values for gausian blur
        pixel_diameter = 9
        sigma_color = 30
        sigma_space = 30

        # bottom offset to account for in-image
        # frame
        bottom_offset = 5

        path = []
        frame = self.bridge.imgmsg_to_cv2(msg)

        # frame processing
        imgGray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        imgGray = cv2.bilateralFilter(frame, pixel_diameter, sigma_color, sigma_space)
        imgEdge = cv2.Canny(
            imgGray, edge_detection_min_threshold, edge_detection_max_threshold
        )

        width = imgEdge.shape[1] - 1
        height = imgEdge.shape[0] - 1

        # iterating throught pixels from (0, height) (bottom left hand corner)
        # to start from position of the camera (rover)
        for x in range(0, width, pixel_steps):
            for y in range(height - bottom_offset, 0, -1):

                # cheching for object's closing edges identified
                # by white pixels
                if imgEdge.item(y, x) == 255:
                    path.append(x)
                    path.append(y)
                    break

            # add pixels all the way to the top (y = 0)
            # if no edges are found
            else:
                path.append(x)
                path.append(0)

        self.publisher_.publish(Int16MultiArray(data=path))
        self.get_logger().info("sent obstruction data")


def main(args=None):
    rclpy.init(args=args)
    publisher = ObstructionPublisher()
    rclpy.spin(publisher)

    feed_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
