<br>
<p align="center">
<h1 align="center"><strong>Fast-UMI: A Scalable and Hardware-Independent Universal Manipulation Interface</strong></h1>
</p>

<div id="top" align="center">

[![Releases](https://img.shields.io/github/release/Zhefan-Xu/CERLAB-UAV-Autonomy.svg)]([https://github.com/Zhefan-Xu/CERLAB-UAV-Autonomy/releases](https://github.com/zxzm-zak/AlignBot/blob/main/README.md))
[![license](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT) 
[![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/20.04/)
[![Hugging Face](https://img.shields.io/badge/Hugging%20Face-FastUMI_Data-blue.svg)](https://huggingface.co/datasets/cpa2001/FastUMI_Data)

[[Project page]](https://fastumi.com/)
[[Huggingface Dataset]](https://huggingface.co/datasets/IPEC-COMMUNITY/FastUMI-Data)
[[PDF (Early Version)]](https://arxiv.org/abs/2409.19499)
[[PDF (TBA)]](https://fastumi.com/)
[[Dataset (TBA)]](https://fastumi.com/)

![](docs/fastumi-data.png)

Welcome to the official repository of FastUMI!

![](docs/fastumi.jpg)

Physical prototypes of Fast-UMI system

</div>

## 📋 Contents

- [📋 Contents](#-contents)
- [🔥 News](#-news)
- [💡 Abstract](#abstract)
- [🏠 Environment Setup](#1-️-environment-setup)
  - [🚀 ROS Installation](#11-ros-installation)
  - [🔭 Python Environment](#12-python-environment)
  - [🛒 Install the necessary ROS packages](#13-install-the-necessary-ros-packages)
  - [📺 Intel RealSense Dependencies](#14-intel-realsense-dependencies)
- [📝 Workflow](#2-workflow)
  - [🌏 Start ROS Core](#21-start-ros-core)
  - [🪢 Connect RealSense T265 and GoPro](#22-connect-realsense-t265-and-gopro)
  - [🎇 Open RViz for Visualization](#23-open-rviz-for-visualization)
  - [🏷️ Run Data Collection Script](#24-run-data-collection-script)
- [📚 Data Processing Details](#3-data-processing-details)
  - [🤝 Trajectory Transformation](#31-trajectory-transformation)
  - [📸 Gripper Width Detection](#32-gripper-width-detection)
  - [🏹 Inverse Kinematics (IK) Computation](#33-inverse-kinematics-ik-computation)


## 🔥 News
- [2024-12] We released Data Collection Code and Dataset.

## 💡 Abstract

Collecting real-world manipulation trajectory data involving robotic arms is essential for developing general-purpose action policies in robotic manipulation, yet such data remains scarce. Existing methods face limitations such as high costs, labor intensity, hardware dependencies, and complex setup requirements involving SLAM algorithms. In this work, we introduce Fast-UMI, an interface-mediated manipulation system comprising two key components: a handheld device operated by humans for data collection and a robot-mounted device used during policy inference. Our approach employs a decoupled design compatible with a wide range of grippers while maintaining consistent observation perspectives, allowing models trained on handheld-collected data to be directly applied to real robots. By directly obtaining the end-effector pose using existing commercial hardware products, we eliminate the need for complex SLAM deployment and calibration, streamlining data processing. Fast-UMI provides supporting software tools for efficient robot learning data collection and conversion, facilitating rapid, plug-and-play functionality. This system offers an efficient and user-friendly tool for robotic learning data acquisition.


## 1. 🏠 Environment Setup

To set up the environment, follow these steps:

### 1.1 🚀 ROS Installation  

Ensure that ROS is installed on your system, see:
    
    http://wiki.ros.org/ROS/Installation

### 1.2 🔭 Python Environment

Create a Virtual Environment and Install Dependencies

    conda create -n FastUMI python=3.8.0
    conda activate FastUMI
    pip install -r requirements.txt

### 1.3 🛒 Install the necessary ROS packages

    sudo apt install python3-catkin-tools python3-rospkg python3-rosdep python3-rosinstall-generator
    sudo apt install ros-<ros-distro>-geometry-msgs ros-<ros-distro>-sensor-msgs ros-<ros-distro>-nav-msgs
    sudo apt install ros-<ros-distro>-cv-bridge
    sudo apt install ros-<ros-distro>-usb-cam

Replace `<ros-distro>` with your ROS distribution, e.g., `noetic`.

### 1.4 📺 Intel RealSense Dependencies
If your system is missing RealSense dependencies, follow these steps:
- Clone and install the [Intel RealSense library](https://github.com/IntelRealSense/librealsense).
- Alternatively, use the following commands to install the necessary libraries:
  ```
  sudo apt install ros-<ros-distro>-realsense2-camera
  sudo apt-key adv --keyserver keys.gnupg.net --recv-key 04A3CF2E
  sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo $(lsb_release -cs) main"
  sudo apt update
  sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg
  ```

---

## 2. 📝 Workflow

### 2.1 🌏 Start ROS Core
Start the ROS master node:

    roscore

### 2.2 🪢 Connect RealSense T265 and GoPro
Launch the RealSense and USB camera nodes:

    roslaunch realsense2_camera rs_t265.launch
    roslaunch usb_cam usb_cam-test.launch

### 2.3 🎇 Open RViz for Visualization
Launch RViz:

    rviz

- Add **Odometry** and **Image** topics to visualize the data from the RealSense and GoPro cameras.

### 2.4 🏷️ Run Data Collection Script
- Update the data saving path in the script.
- Run the data collection script:

      python data_collection.py

Alternatively, use the provided shell script.

---

## 3. 📚 Data Processing Details
### 3.1 🤝 Trajectory Transformation
- The raw data consists of the RealSense T265 trajectory.
- An **offset** is added to transform the T265 trajectory into the TCP (Tool Center Point) trajectory.
- Coordinate transformations are applied during this process to map the T265 trajectory to the TCP trajectory.
- The `offset` field in the `config.json` file contains the offsets for the T265 to TCP transformation in the x-axis and z-axis directions.

### 3.2 📸 Gripper Width Detection
- The gripper width is detected using ArUco markers attached to the gripper.
- If any frame fails to detect all the markers, the gripper width is interpolated to ensure complete data.
- The `distances` field in the `config.json` file contains the maximum and minimum pixel distances of the markers in the image (`marker_max` and `marker_min`), as well as the actual maximum gripper width of the robot (`gripper_max`).

### 3.3 🏹 Inverse Kinematics (IK) Computation
- A simple IK computation is implemented to convert TCP data into the corresponding absolute joint angles of the robot arm, which facilitates ACT model training.
- To calculate IK, the distance from the TCP to the flange must be obtained. The `distances` field in the `config.json` file includes this distance as `flange_to_tcp`.
