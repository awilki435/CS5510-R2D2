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


class Planner(Node):
    """Make and execute a plan."""
    def __init__(self):
        """Initialize Node."""
        super().__init__(node_name="Planner")
        self.get_logger().info("Initializing...")

        # Create callback groups
        self.cb_estimate = MutuallyExclusiveCallbackGroup()
        self.cb_velocity = MutuallyExclusiveCallbackGroup()

        # TODO: Create client

        # Receive estimated position
        self.pose_sub = self.create_subscription(PoseStamped, "pose_estimate", self.pose_estimate_callback, 10, callback_group=self.cb_estimate)

        # Publish velocity commands
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10, callback_group=self.cb_velocity)

        # Inputs
        self.v_r = 0.0
        self.v_l = 0.0
        # States
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        # Goal condition
        self.goal_set = False
        self.goal_reached = False
        self.x_g = 0.0
        self.y_g = 0.0
        self.theta_g = 0.0
        self.distance_to_goal = 0.0
        # Parameters
        self.sphere_of_influence = 0.0
        self.radius_of_influence = 0.0
        self.max_vel = 0.0
        # Robot specs
        self.wheel_radius = 0.0
        self.axle_length = 0.0

    def pose_estimate_callback(self, msg: PoseStamped):
        """Save Position Estimate"""
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        # Find theta using w and z
        w = msg.pose.orientation.w
        z = msg.pose.orientation.z
        self.theta = 2 * math.atan2(z,w)
        if self.goal_set and not self.goal_reached:
            self.distance_to_goal = math.dist([self.x, self.y],[self.x_g, self.y_g])

    def goal_callback(self, msg: PoseStamped):
        self.x_g = msg.pose.position.x
        self.y_g = msg.pose.position.y
        # Calcluate theta using w and z
        w = msg.pose.orientation.w
        z = msg.pose.orientation.z
        self.theta_g = 2 * math.atan2(z,w)
        self.get_logger().info("Goal has been set")
        self.goal_set = True
        self.goal_reached = False

    def publish_velocity(self):
        """Send velocity commands to the wheels"""
        if self.goal_set is True:
            # Create vector for go to goal control
            distance_vector = [self.x_g - self.x , self.y_g - self.y]
            scale = 0.0
            if self.distance_to_goal > self.sphere_of_influence:
                scale = 1.0
            elif self.radius_of_influence < self.distance_to_goal <= self.sphere_of_influence:
                scale = 1 - (self.sphere_of_influence - self.distance_to_goal) / (self.sphere_of_influence - self.radius_of_influence)
            elif self.distance_to_goal <= self.radius_of_influence:
                scale = 0.0
            v_g = self.max_vel * scale
            if self.distance_to_goal > 0:
                distance_vector *= v_g/self.distance_to_goal
            else:
                distance_vector = [0,0]
            
            # Unicycle Model with Proportional Control
            v = np.linalg.norm(np.array(distance_vector))
            k_gain = 0.75
            theta_d = math.atan2(distance_vector[1],distance_vector[0])
            omega = -k_gain * (self.theta - theta_d)

            # Convert to Differential Drive
            inputs = np.array([[v],[omega]])
            r = self.wheel_radius
            L = self.axle_length
            transform = np.linalg.inv(np.array([[[r/2],[r/2]],[[r/L],[-r/L]]]))
            self.v_r, self.v_l = transform@inputs

            # Create and send message
            msg = Twist()
            msg.linear.x = self.v_r
            msg.linear.y = self.v_l
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            self.vel_pub.publish(msg)
        else:
            # Create and send message
            msg = Twist()
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
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

if __name__ == '__main__':
    main()
