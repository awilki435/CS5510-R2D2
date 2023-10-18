# import typing
import typing
import rclpy
import os
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2



class ImageRecorder(Node):
    """Record camera images in a png format."""
    def __init__(self):
        super().__init__(node_name="ImageRecorder")
        self.get_logger().info("Initializing...")

        # Create callback groups
        cb_timer = MutuallyExclusiveCallbackGroup()
        cb_image = MutuallyExclusiveCallbackGroup()

        # Create timer for saving an image to a file
        self.create_timer(timer_period_sec=0.5, callback=self.save_file,callback_group=cb_timer)

        # Create a subscriber for an image message
        self.sub_image = self.create_subscription(
            Image, "raw_image", self.image_callback, 10, callback_group=cb_image
        )
        # Create a storage variables
        self.latest_image: typing.Optional[Image] = None
        self.index = 0
        self.directory = "./src/visual_odometry/images"

    def image_callback(self, msg: Image) -> None:
        """Save latest message."""
        self.latest_image = msg

    def save_file(self) -> None:
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(self.latest_image, desired_encoding="passthrough")
        # Get the absolute path of the directory
        abs_dir_path = os.path.join(os.getcwd(), self.directory)

        # Check if the directory exists, if not, create it
        if not os.path.exists(abs_dir_path):
            os.makedirs(abs_dir_path)

        # Now you can save your file in this directory
        filename = os.path.join(abs_dir_path, self.update_filename())
        cv2.imwrite(filename, img)


    def update_filename(self) -> str:
        """Update the filename due to different indices."""
        filename = ""
        self.index += 1
        if self.index < 10:
            filename = f"00000{self.index}.png" 
        elif self.index < 100:
            filename = f"0000{self.index}.png" 
        elif self.index < 1000:
            filename = f"000{self.index}.png" 
        elif self.index < 10000:
            filename = f"00{self.index}.png" 
        elif self.index < 100000:
            filename = f"0{self.index}.png" 
        elif self.index < 1000000:
            filename = f"{self.index}.png" 
        else:
            self.get_logger().fatal("Number of image files exceeded limit")
        return filename

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
