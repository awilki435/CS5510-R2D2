"""Localizer Node."""

# Takes in video data
# figure out how far the ball/goal is
# send a PoseStamped message to the planner

import typing
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from geometry_msgs.msg._pose_stamped import PoseStamped


class Localizer(Node):
    """Localize where the ball and goal are."""

    def __init__(self):
        """Initialize Node."""
        super().__init__(node_name="Localizer")
        self.get_logger().info("Initializing...")

        # create callback groups
        self.cb_image = MutuallyExclusiveCallbackGroup()
        self.cb_goal = MutuallyExclusiveCallbackGroup()
        self.cb_reached = MutuallyExclusiveCallbackGroup()

        # subscribe to the image message and store the latest message
        self.sub_image = self.create_subscription(
            Image, "raw_image", self.image_callback, 10, callback_group=self.cb_image
        )
        self.first_image: typing.Optional[Image] = None
        self.image_taken = False

        # find and publish ball/goal
        self.goal_pub = self.create_publisher(
            PoseStamped, "goal_pose", 10, callback_group=self.cb_goal
        )
        self.image_processing()

    def image_callback(self, msg: Image) -> None:
        """Save latest message."""
        if self.image_taken == False:
            self.first_image = msg
            self.image_taken = True

    def image_processing(self):
        """Process the image."""
        # Find ball and goal and see if they can be found
        # while ball or goal not found
        #   set image_taken to False to take in new image
        # Do the trig
        # publish where locations are and then die

        self.find_ball()
        self.find_goal()


    def find_ball(self) -> PoseStamped:
        """Find where the ball is."""
        # Read the captured image
        # TODO: Change the image to an image captured by the robot
        image = cv2.imread('ball_moving_robot/balls.jpg')

        # Convert the image to the HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        self.get_logger().info("Finding ball...")
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        largest_contour = max(contours, key=cv2.contourArea)

        # Calculate the centroid of the largest contour
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            #Testing:
            # Print or use the x, y coordinates of the center centroid
            #print(f"Center Centroid: x={cx}, y={cy}")

            # Optionally, draw a circle at the center centroid on the original image
            #cv2.circle(image, (cx, cy), 5, (0, 255, 0), -1)
        # Display the original image with centroids marked
        #cv2.namedWindow('Original Image with Centroids', cv2.WINDOW_NORMAL)
        #cv2.imshow('Original Image with Centroids', image)
        msg = PoseStamped()
        msg.pose.position.x = cx
        msg.pose.position.y = cy
        msg.header.frame_id = "map"
        self.goal_pub.publish(msg)

    def find_goal(self) -> PoseStamped:
        """Find where the goal is."""
        # Read the captured image
        # TODO: Change the image to each image captured by the robot
        image = cv2.imread('ball_moving_robot/balls.jpg')

        # Convert the image to the HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        self.get_logger().info("Finding ball...")
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([80, 255, 255])
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        largest_contour = max(contours, key=cv2.contourArea)

        # Calculate the centroid of the largest contour
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            #Testing:
            # Print or use the x, y coordinates of the center centroid
            #print(f"Center Centroid: x={cx}, y={cy}")

            # Optionally, draw a circle at the center centroid on the original image
            #cv2.circle(image, (cx, cy), 5, (0, 255, 0), -1)
        #Testing: 
        # Display the original image with centroids marked
        #cv2.namedWindow('Original Image with Centroids', cv2.WINDOW_NORMAL)
        #cv2.imshow('Original Image with Centroids', image)
        self.get_logger().info("Finding goal...")
        msg = PoseStamped()
        msg.pose.position.x = cx
        msg.pose.position.y = cy
        msg.header.frame_id = "map"
        self.goal_pub.publish(msg)


def main(args=None):
    """Spin the node."""
    # Initialize ros
    rclpy.init(args=args)

    # Create node and executor
    node = Localizer()
    exec = MultiThreadedExecutor()
    exec.add_node(node)
    exec.spin()


if __name__ == "__main__":
    main()
