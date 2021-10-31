# ORB-SLAM 2 robot_localization

"A Generalized Extended Kalman Filter Implementation for the Robot Operating System" paper available at: http://docs.ros.org/en/melodic/api/robot_localization/html/_downloads/robot_localization_ias13_revised.pdf 

## 1. Source code
### - Prerequisites:

[1] To run on the Bebop 2 drone install Bebop Autonomy: https://bebop-autonomy.readthedocs.io/en/latest/
    
[2] You will also need to calibrate the drone camera for ORB-SLAM 2. Calibration instructions available at    https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html

[3] ORB-SLAM2 ROS node from https://github.com/appliedAI-Initiative/orb_slam_2_ros. Please follow the build instructions there

### - How to build (Ubuntu 16.04, ROS Kinetic):
[1] Install robot_localization package:
````
sudo apt-get install ros-kinetic-robot-localization
````
[2] Clone this repository first:
````
git clone https://github.com/SebMay99/modified_stability_scale_estimation
````
[3] Build:
````
cd modified_stability_scale_estimation/catkin_ws
catkin_make
````


