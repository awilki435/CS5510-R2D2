"""Localizer Node."""

# Takes in video data
# figure out how far the ball/goal is
# send a PoseStamped message to the planner

import typing
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from geometry_msgs.msg._pose_stamped import PoseStamped

class Localizer(Node):
    """Localize where the ball and goal are"""
    def __init__(self):
        """Initialize Node."""
        super().__init__(node_name="Localizer")
        self.get_logger().info("Initializing...")

        # create callback groups
        self.cb_image = MutuallyExclusiveCallbackGroup()

        # subscribe to the image message and store the latest message
        self.sub_image = self.create_subscription(
            Image, "raw_image", self.image_callback, 10, callback_group=self.cb_image
        )
        self.latest_image: typing.Optional[Image] = None
        
        # TODO: Create Service for finding ball or goal

    def image_callback(self, msg: Image) -> None:
        """Save latest message."""
        self.latest_image = msg

    def find_ball(self) -> PoseStamped:
        """Find where the ball is."""
        pass

    def find_goal(self) -> PoseStamped:
        """Find where the goal is."""
        pass

def main(args=None):
    """Spin the node."""
    # Initialize ros
    rclpy.init(args=args)

    # Create node and executor
    node = Localizer()
    exec = MultiThreadedExecutor()
    exec.add_node(node)
    exec.spin()

if __name__ == '__main__':
    main()
