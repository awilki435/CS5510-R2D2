"""Driver Node."""

# import smbus
import math
import numpy as np
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
        self.get_logger().info("Initializing Driver...")

        # variables to move the robot
        self._addr = 0x16
        # self._device = smbus.SMBus(1)
        self.v = 0.0
        self.omega = 0.0

        #define vehicle parameters
        self.wheel_radius = 0.05
        self.axle_length = 0.1

        # Create callback groups
        self.cb_move = MutuallyExclusiveCallbackGroup()
        self.cb_velocity = MutuallyExclusiveCallbackGroup()

        timer_period = 0.1
        self.pose_timer = self.create_timer(
            timer_period, self.move_robot, callback_group=self.cb_move
        )

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
        self.v = msg.linear.x
        self.omega = msg.angular.z

    def move_robot(self):
        """Move the robot.
        left: int (-255, 255)
        right: int (-255, 255)

        sets the motor with the speed given (not actually in unit, just a power amount)
        """

        # Convert to Differential Drive:
        inputs = np.array([[self.v], [self.omega]])
        r = self.wheel_radius
        L = self.axle_length
        transform = np.linalg.inv(np.array([[r / 2, r / 2], [r / L, -r / L]]))
        right, left = transform @ inputs

        # Scale magnitude to be between -255 and 255
        max_speed = max(abs(right), abs(left))
        if max_speed > 255:
            right = right / max_speed * 255
            left = left / max_speed * 255

        self.v_r = float(right)
        self.v_l = float(left)
        
        # Controls to the actual robot
        register = 0x01
        left_direction = 0 if left < 0 else 1
        right_direction = 0 if right < 0 else 1

        if left < 0:
            left *= -1
        if right < 0:
            right *= -1

        data = [left_direction, left, right_direction, right]
        try:
            # smbus.SMBus(1).write_i2c_block_data(self._addr, register, data)
            print("left: ", left, "right: ", right)
        except:
            self.get_logger().error("write array error in control_car")


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
