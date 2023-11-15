"""Driver Node."""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# from Car import Car

# inputs: v and w
# outputs: discretized values of power to the wheels

class Driver(Node):
    """Move the robot towards a goal."""
    def __init__(self):
        """Initialize Node."""
        super().__init__(node_name="Driver")
        self.get_logger().info("Initializing...")

        # TODO: get Car to work
        # self.commands = Car()

        # TODO: create publisher for estimation
        # TODO: create commands to wheels

    def velocity_callback(self):
        pass

def main(args=None):
    """Spin the node."""
    # Initialize ros
    rclpy.init(args=args)

    # Create node and executor
    node = Driver()
    exec = MultiThreadedExecutor()
    exec.add_node(node)
    exec.spin()

if __name__ == "__main__":
    main()