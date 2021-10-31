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
git clone https://github.com/SebMay99/ORB-SLAM2_robot_localization
````
[3] Build:
````
cd ORB-SLAM2_robot_localization/catkin_ws/
catkin_make
````

## 2. How to run
### 1. Launch Bebop Autonomy driver
Connect to the drone
````
roslaunch bebop_driver bebop_node.launch
````
### 2. Run ORB-SLAM2
Note that the monocular node subscribes to a topic `/camera/image_raw` to run node ORB_SLAM2/Mono. So you need to relay to your camera topic
````
- Launch ORB-SLAM 2 Mono:
roslaunch orb_slam2_ros [your_launch_file].launch

- Visualize ORB SLAM 2 debug image
rosrun image_view image_view image:=/orb_slam2_mono/debug_image

- You may want to change the calib.yaml to your own calibration file:
cd ~/orb_slam_2_ros/orb_slam2/config/[your_calibration_file].yaml

````
### 3. Add the covariance matrix to the ORB-SLAM 2 pose
Run `covariance_matrix_pose.py` to modifiy the SLAM pose by adding a covariance matrix. The new pose will be published in the `processed_pose` topic.

### 4. Run the EKF
Launch the EKF node, the parameters can be adjusted in the `ekf_localization.yaml` file.
````
cd catkin_ws/ && source devel/setup.bash 
roslaunch summit_odometry start_filter.launch
````
You can read the filtered pose in a new terminal:
````
rostopic echo odometry/filtered
````
