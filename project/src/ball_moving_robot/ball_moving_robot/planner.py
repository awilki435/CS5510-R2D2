"""Planner Node."""

# Take in a PoseStamped message from localizer
# create a go-to-goal control
# send lateral and angular velocity commands to the driver

import typing
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg._pose_stamped import PoseStamped


class Planner(Node):
    """Make and execute a plan."""
    def __init__(self):
        """Initialize Node."""
        super().__init__(node_name="Planner")
        self.get_logger().info("Initializing...")

        # Create callback groups
        self.cb_estimate = MutuallyExclusiveCallbackGroup()

        # TODO: Create client
        self.goal: typing.Optional[PoseStamped] = None

        # Find estimated position
        self.create_subscription(PoseStamped, "pose_estimate", self.pose_estimate, 10, callback_group=self.cb_estimate)
        self.estimated_position: typing.Optional[PoseStamped] = None

    def goal_callback(self, msg: PoseStamped):
        self.goal = msg

    def pose_estimate(self, msg: PoseStamped):
        self.estimated_position = msg

    def publish_velocity(self):
        x_goal = self.goal.pose.position.x
        y_goal = self.goal.pose.position.y
        x_est = self.estimated_position.pose.position.x
        y_est = self.estimated_position.pose.position.x
        # TODO: Look through gtg control and cmd_vel topic

def main(args=None):
    """Spin the node."""
    # Initialize ros
    rclpy.init(args=args)

    # Create node and executor
    node = Planner()
    exec = MultiThreadedExecutor()
    exec.add_node(node)
    exec.spin()

if __name__ == '__main__':
    main()
