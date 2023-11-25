"""Driver Node."""

# import smbus
import math
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, Twist

# from Car import Car

# inputs: v and w
# outputs: discretized values of power to the wheels


class Driver(Node):
    """Move the robot towards a goal."""

    def __init__(self):
        """Initialize Node."""
        super().__init__(node_name="Driver")
        self.get_logger().info("Initializing...")

        # variables to move the robot
        self._addr = 0x16
        # self._device = smbus.SMBus(1)
        self.left = 0.0
        self.right = 0.0

        # Create callback groups
        self.cb_estimate = MutuallyExclusiveCallbackGroup()
        self.cb_velocity = MutuallyExclusiveCallbackGroup()

        # create publisher for estimation
        self.pose_estimate = PoseStamped()
        self.pose_estimate.pose.position.x = 0.0
        self.pose_estimate.pose.position.y = 0.0
        self.pose_estimate.header.frame_id = "map"

        timer_period = 0.1
        self.pose_timer = self.create_timer(
            timer_period, self.move_robot, callback_group=self.cb_estimate
        )
        self.pose_pub = self.create_publisher(PoseStamped, "pose_estimate", 10)

        # create subscriber for velocity commands
        self.vel_sub = self.create_subscription(
            Twist,
            "cmd_vel",
            self.velocity_callback,
            10,
            callback_group=self.cb_velocity,
        )

    def velocity_callback(self, msg: Twist):
        """Receive velocity commands."""
        self.left = msg.linear.y
        self.right = msg.linear.x

    def estimate(self):
        """Estimate the position of the robot."""
        # Euler integration:
        # y(t+dt) = y(t) + dt*f(t)
        r = 2
        d = 4
        dt = 0.1
        # theta = dt * r / d * (self.right - self.left)
        # self.pose_estimate.pose.orientation.z += theta

        # self.pose_estimate.pose.position.x += (
        #     (dt * r / 2)
        #     * (self.right + self.left)
        #     * math.cos(self.pose_estimate.pose.orientation.z)
        # )
        # self.pose_estimate.pose.position.y += (
        #     (dt * r / 2)
        #     * (self.right + self.left)
        #     * math.sin(self.pose_estimate.pose.orientation.z)
        # )
        # self.get_logger().info(f"{theta}")

        # z = math.sin(theta / 2.0)
        # w = math.cos(theta / 2.0)

        # self.pose_estimate.pose.orientation.w += math.cos(theta)

        # Point model
        self.pose_estimate.pose.position.x += dt * (self.left)
        self.pose_estimate.pose.position.y += dt * (self.right)
        self.pose_pub.publish(self.pose_estimate)

    def move_robot(self):
        """Move the robot"""
        self.control_car(self.left, self.right)
        self.estimate()

    def control_car(self, left, right):
        """Control the vehicle.
        left: int (-255, 255)
        right: int (-255, 255)

        sets the motor with the speed given (not actually in unit, just a power amount)
        """

        # register = 0x01
        # left_direction = 0 if left < 0 else 1
        # right_direction = 0 if right < 0 else 1

        # if left < 0:
        #     left *= -1
        # if right < 0:
        #     right *= -1

        # data = [left_direction, left, right_direction, right]
        # try:
        #     self._device.write_i2c_block_data(self._addr, register, data)
        # except:
        #     self.get_logger().error("write array error in control_car")


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
