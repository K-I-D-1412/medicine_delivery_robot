# Visual Navigation for Dense Cone Path

![ROS](https://img.shields.io/badge/ROS-Noetic-orange) ![OpenCV](https://img.shields.io/badge/OpenCV-4.x-green) ![Language](https://img.shields.io/badge/C++-11-blue)

## Introduction
This project implements a robot vision system using **ROS** and **OpenCV**. The robot navigates autonomously through a path defined by dense traffic cones, detects specific targets (medicine pills) at intersections, and makes turning decisions based on the target type.

<p align="center">
  <img src="IMG/robot.gif" alt="RobotDemo" />
</p>

## Features
* **Cone Detection**: Detects red traffic cones using HSV color space and contour analysis.
* **Path Planning**: Calculates the navigation angle based on the center points of the cones.
* **Target Recognition**: Identifies "pills" (blue/green patterns) and counts them to decide turning direction (Left/Right).
* **State Machine**: robust logic handling (Forward -> Check -> Turn -> Stop).

## Hardware & Software
* **Hardware**: 
    * Intel RealSense Camera (or ZED/Webcam)
    * Mobile Robot Platform (supports `cmd_vel` control)
* **Software**:
    * Ubuntu 20.04
    * ROS Noetic
    * OpenCV 4.x

## Installation & Build

1.  **Clone the repository**
    ```bash
    mkdir -p catkin_ws/src
    cd catkin_ws/src
    git clone [https://github.com/YourUsername/HIT-Cone-Navigation.git](https://github.com/YourUsername/HIT-Cone-Navigation.git)
    ```

2.  **Build the package**
    ```bash
    cd ..
    catkin_make
    source devel/setup.bash
    ```

## Usage

1.  **Launch the camera node** (Ensure your RealSense driver is running)
2.  **Run the navigation node**:
    ```bash
    rosrun cone_navigation img_node
    ```

## Algorithm Details
The core algorithm relies on **Color Segmentation** and **Logic Control**:

<p align="center">
  <img src="IMG/screen.gif" alt="ScreenDemo" />
</p>

1.  **Preprocessing**: Gaussian Blur & Histogram Equalization.
2.  **Cone Detection**: 
    * Combines two red masks in HSV space.
    * Filters noise by contour area (>300).
3.  **Decision Making**:
    * Counts blue vs. green connected components to identify the pill type.
    * Uses a fault-tolerant comparison `abs(num_blue-6) < abs(num_green-8)` to handle noise.

## Authors
* **Wang Pengyu**
* **Li Weiqi**
* *Harbin Institute of Technology, Shenzhen*

## License
This project is licensed under the MIT License.
