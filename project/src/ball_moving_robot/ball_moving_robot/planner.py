"""Planner Node."""

# Take in a PoseStamped message from localizer
# create a go-to-goal control
# send lateral and angular velocity commands to the driver

import typing

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool


class Planner(Node):
    """Make and execute a plan."""

    def __init__(self):
        """Initialize Node."""
        super().__init__(node_name="Planner")
        self.get_logger().info("Initializing...")

        # Create callback groups
        self.cb_estimate = MutuallyExclusiveCallbackGroup()
        self.cb_velocity = MutuallyExclusiveCallbackGroup()
        self.cb_goal = MutuallyExclusiveCallbackGroup()
        self.cb_reached = MutuallyExclusiveCallbackGroup()

        # Receive goal position
        self.goal_sub = self.create_subscription(
            PoseStamped,
            "goal_pose",
            self.goal_callback,
            10,
            callback_group=self.cb_goal,
        )

        # Publish estimated position
        timer_period = 0.1
        self.estimate_timer = self.create_timer(
            timer_period, self.estimate, callback_group=self.cb_estimate
        )
        self.pose_pub = self.create_publisher(PoseStamped, "pose", 10)

        # Publish velocity commands
        self.velocity_timer = self.create_timer(
            timer_period, self.publish_velocity, callback_group=self.cb_velocity
        )
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # position and goals
        self.goals: typing.List[PoseStamped] = []
        self.pose = PoseStamped()
        self.distance_to_goal = np.inf
        self.sphere_of_influence = 7.5
        self.radius_of_influence = 0.01
        self.max_vel = 2
        # states
        self.x = -5.0
        self.y = 0.0
        self.theta = 0.0
        # velocity commands
        self.v = 0.0
        self.omega = 0.0


    def goal_callback(self, msg: PoseStamped):
        """Save Goal Location."""
        self.goals.append(msg)
        self.get_logger().info("Goal has been set")

    def estimate(self):
        """Estimate position of the robot."""
        if len(self.goals) == 0:
            return
        # Euler integration:
        # y(t+dt) = y(t) + dt*f(t)
        dt = 0.1
        self.x = self.x + dt * self.v * np.cos(self.theta)
        self.y = self.y + dt* self.v * np.sin(self.theta)
        self.theta = self.theta + dt * self.omega

        goal = self.goals[0]
        x_g = goal.pose.position.x
        y_g = goal.pose.position.y

        self.distance_to_goal = math.dist([self.x, self.y],[x_g,y_g])
        pose_estimate = PoseStamped()
        pose_estimate.pose.position.x = self.x
        pose_estimate.pose.position.y = self.y
        # abuse notation
        pose_estimate.pose.orientation.z = self.theta
        pose_estimate.header.frame_id = "map"
        self.pose_pub.publish(pose_estimate)

    def publish_velocity(self):
        """Send velocity commands to the wheels."""
        if len(self.goals) == 0:
            # stay still
            msg = Twist()
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            self.vel_pub.publish(msg)
            return
        # Create vector for go to goal control
        goal = self.goals[0]
        x_g = goal.pose.position.x
        y_g = goal.pose.position.y
        distance_vector = [x_g - self.x, y_g - self.y]
        scale = 0.0
        if self.distance_to_goal > self.sphere_of_influence:
            scale = 1.0
        elif (
            self.radius_of_influence < self.distance_to_goal <= self.sphere_of_influence
        ):
            scale = 1 - (self.sphere_of_influence - self.distance_to_goal) / (
                self.sphere_of_influence - self.radius_of_influence
            )
        elif self.distance_to_goal <= self.radius_of_influence:
            scale = 0.0
        v_g = self.max_vel * scale
        if self.distance_to_goal > 0.1:
            distance_vector = [
                distance_vector[0] * v_g / self.distance_to_goal,
                distance_vector[1] * v_g / self.distance_to_goal,
            ]
        else:
            distance_vector = [0, 0]
            self.get_logger().info("Goal Reached")
            self.goals.pop(0)

        # Unicycle Model with Proportional Control
        self.v = np.linalg.norm(np.array(distance_vector))
        k_gain = 0.75
        theta_d = math.atan2(distance_vector[1], distance_vector[0])
        self.omega = -k_gain * (self.theta - theta_d)

        # Create and send message
        msg = Twist()
        msg.linear.x = self.v
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.omega
        self.vel_pub.publish(msg)


def main(args=None):
    """Spin the node."""
    # Initialize ros
    rclpy.init(args=args)

    # Create node and executor
    node = Planner()
    exec = MultiThreadedExecutor()
    exec.add_node(node)
    exec.spin()


if __name__ == "__main__":
    main()
