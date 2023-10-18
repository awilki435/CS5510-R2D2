# Homework 2 Instructions for Running

This document describes how to build and run the necessary functionality for the assignment.

## ROS Package Installation
This assumes that you have ROS Humble on your computer and are running Ubuntu 22.04. It also assumes that you have OpenCv installed on your computer.

Go to the workspace root (`HW_02`) and run the following command in your terminal:
```
source /opt/ros/humble/setup.bash && python3 -m colcon build --symlink-install && . install/local_setup.bash
```
This sources ROS, builds the package, and makes it so that you can run the nodes in the package. To test the functionality of the package, run

```
ros2 run visual_odometry test_pub
```
in your terminal. In a second terminal, run

```
source /opt/ros/humble/setup.bash && ros2 run visual_odometry image_recorder
```

This sources ROS and runs the image recorder. As you can see, the random image generated from the test publisher is read and saved by the `image_recorder`.