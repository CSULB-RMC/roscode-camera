import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2 as cv

from flask import Flask, render_template, Response
from threading import Thread, Event


# flask app
app = Flask(__name__)

# image frame
frame = None

# Threading event used to keep the stream going (non-blocking)
event = Event()

WEB_SERVER_HOST = "127.0.0.1"
WEB_SERVER_PORT = "5000"


class FeedSubscriber(Node):
    def __init__(self):
        super().__init__("subscriber_feed")

        self.subscription = self.create_subscription(
            Image, "camera_feed", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        """listen for image data"""
        global frame

        self.get_logger().info(
            f"streaming at http://{WEB_SERVER_HOST}:{WEB_SERVER_PORT}"
        )

        frame = self.bridge.imgmsg_to_cv2(msg)
        ret, buffer = cv.imencode(".jpg", frame)
        frame = buffer.tobytes()

        # reset event to allow thread execution
        event.set()


def get_frame():
    """get the most recent frame non blocking
    (only gets the frame if the event allows it)
    """
    event.wait()
    event.clear()
    return frame


def fetch_frame():
    """fetch last saved frame"""
    while True:
        frame = get_frame()
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
        )


@app.route("/video-feed")
def video_feed():
    """get video feed as image files"""
    return Response(
        fetch_frame(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/")
def index():
    """Index route"""
    # docker can't find the template file, so as a temporary fix,
    # the html for the site is hardcoded below
    # return render_template("index.html", name=None)
    return """
    <!DOCTYPE html>
    <html>
      <head>
        <meta charset="UTF-8" />
        <meta name="viewport" content="width=device-width, initial-scale=1.0" />
        <script src="https://cdn.tailwindcss.com"></script>
        <title>Rover</title>
      </head>
      <body>
        <div class="flex flex-col items-center my-2">
          <img
            class="w-1/12 mb-4"
            src="https://beachlunabotics.org/wp-content/uploads/2021/05/LBL_Logo.png"
          />
          <img class="w-3/5 aspect-video" src="/video-feed" />
        </div>
      </body>
    </html>
    """


def main(args=None):
    rclpy.init(args=args)
    feed_subscriber = FeedSubscriber()

    # adding subscriber on a sub thread so that it won't interfere with
    # server process
    Thread(target=lambda: rclpy.spin(feed_subscriber)).start()

    # starting web server
    app.run(host=WEB_SERVER_HOST, port=WEB_SERVER_PORT, debug=True)

    feed_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
