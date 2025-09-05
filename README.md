# Robotics-Engineering
Rover localization in ROS using IMU, wheel encoders, and GPS RTK. Data is fused with an Extended Kalman Filter (EKF) for accurate pose estimation. Results are visualized in RViz for reliable navigation and real-time monitoring.
Rover Odometry and Sensor Fusion with ROS
📌 Overview

This package provides a complete implementation for rover localization and visualization using ROS. The rover is equipped with an IMU (BNO055) and wheel encoders, which provide raw orientation and odometry data. These measurements are fused using an Extended Kalman Filter (EKF) to generate accurate pose estimation. The results are visualized in RViz for real-time monitoring.


🚀 Features

IMU node for orientation data

Encoder node for wheel odometry

EKF node for sensor fusion

Real-time visualization in RViz

Modular ROS structure for easy integration


#installation

# Clone the repository into your ROS workspace
cd ~/catkin_ws/src
git clone https://github.com/hishamsaif123/llaunch_jetson_nano_bot.git

# Build the workspace
cd ~/catkin_ws
catkin_make
source devel/setup.bash


📊 Visualization

Odometry (from encoders)

Orientation (from IMU)

Fused pose (EKF output)


🛠 Requirements

ROS (Noetic / Melodic)

robot_localization package (for EKF)

RViz
