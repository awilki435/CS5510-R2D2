# import typing
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge



class ImageRecorder(Node):
    """Record camera images in a png format."""
    def __init__(self):
        super().__init__(node_name="ImageRecorder")
        self.get_logger().info("Initializing...")

        # Create callback groups
        cb_timer = MutuallyExclusiveCallbackGroup()

        # Create timer for saving an image to a file
        self.create_timer(timer_period_sec=0.09, callback=self.send_image,callback_group=cb_timer)

        # Create a subscriber for an image message
        self.pub_image = self.create_publisher(Image, "images", 10)

    def send_image(self) -> None:
        """Send a random image."""
        # Create a 32 by 32 numpy ndarray with random values between 0 and 255
        random_array = np.random.randint(0, 256, size=(100, 100, 3), dtype=np.uint8)

        # Convert the numpy ndarray to an OpenCV image
        cv_image = cv2.cvtColor(random_array, cv2.COLOR_BGR2RGB)

        # Convert the OpenCV image to a ROS2 message
        bridge = CvBridge()
        ros_image = bridge.cv2_to_imgmsg(cv_image, encoding='rgb8')
        self.pub_image.publish(ros_image)

def get_identity(limit: int):
    identity = []
    for i in range(limit):
        row = []
        for j in range(limit):
            if i == j:
                row.append(1)
            else:
                row.append(0)
        identity.append(row)

    return identity


def main(args=None):
    """Run the node."""
    # Initialize ROS
    rclpy.init(args=args)
    # Create node
    node = ImageRecorder()
    exec = SingleThreadedExecutor()
    exec.add_node(node)
    exec.spin()

if __name__ == '__main__':
    main()
