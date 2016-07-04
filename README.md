# arm_slam_calib

This experimental package tracks and calibrates a robot arm using monocular visual SLAM. It was created as a proof of concept for my (Matt Klingensmith's) thesis. It provides the following:

* Joint angle estimates for the robot's trajectory
* The extrinsic calibration of a monocular camera with respect to the robot's body

It is designed for serial-link robot manipulators with a single monocular camera on the end effector or some other part of the robot's body. The user must provide the robot's input. The SLAM system reads the robot's joint angles and images from the camera through ROS. Visualizations are provided through ROS. Depth images (for instance from Kinect-like projective light sensors) may be used for visualization, but are not used for calibration/tracking.

#Requirements#
* A serial link robot manipulator emitting joint states through ROS.
* A monocular camera that has been intrinsically calibrated, emitting RGB images and intrinsic calibration through ROS
* [URDF file](http://wiki.ros.org/urdf) for your robot
* ROS indigo or higher (probably on Linux, preferably Ubuntu)
* catkin build system

##ROS Packages##
* geometry_msgs
* cv_bridge
* sensor_msgs
* std_msgs
* roscpp
* image_transport
* visualization_msgs
* pcl_ros
* [apriltags](https://github.com/personalrobotics/apriltags)
* [joint_state_recorder](https://github.com/personalrobotics/joint_state_recorder)

##Non-ROS Dependencies##
* [AIKIDO](https://github.com/personalrobotics/aikido)
* [DART](https://github.com/dartsim/dart)
* [BRISK](https://github.com/clemenscorny/brisk)
* [OpenCV](https://github.com/opencv/opencv)
* [OpenGV](https://github.com/laurentkneip/opengv)
* [GTSAM](https://github.com/devbharat/gtsam)
* [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page)
* [PCL](https://github.com/PointCloudLibrary/pcl)
* [Assimp](https://github.com/assimp/assimp)
* [TBB](https://github.com/wjakob/tbb)
* C++11

# Usage #
The tracking/calibration algorithm can be configured to run with or without fiducials. 

To use fiducials, you will have to first get an [April Tags](https://github.com/personalrobotics/apriltags) node up and running, observing april tags in the environment. The SLAM system can use the april tags messages directly. It will also subscribe to the color images (or depth images, where available) for debugging purposes. 

The system can also be run entirely without fiducials, using the texture in the environment from [BRISK](https://github.com/clemenscorny/brisk) features. Make sure that the robot is in a highly textured, static environment. [Calibrate the intrinsics](http://wiki.ros.org/openni_launch/Tutorials/IntrinsicCalibration) of your camera first. They must be output to a `CameraInfo` topic through ROS. The images can be of any format. `BGR8` and `mono8` have been tested. Your mileage may vary.
