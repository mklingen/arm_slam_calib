# arm_slam_calib

This experimental package tracks and calibrates a robot arm using monocular visual SLAM. It provides the following:

* Joint angle estimates for the robot's trajectory
* The extrinsic calibration of a monocular camera with respect to the robot's body

It is designed for serial-link robot manipulators with a single monocular camera on the end effector or some other part of the robot's body. The user must provide the robot's input. The SLAM system reads the robot's joint angles and images from the camera through ROS. Visualizations are provided through ROS. Depth images (for instance from Kinect-like projective light sensors) may be used for visualization, but are not used for calibration/tracking.
