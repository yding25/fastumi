# README

## 1. Environment Setup

To set up the environment, follow these steps:

### 1.1 Install Required Dependencies
1. **ROS Installation**  
    Ensure that ROS is installed on your system, see:
    
    http://wiki.ros.org/ROS/Installation

2. **Python Environment**  
    Use Python version `3.8.19`. Install the required Python packages:

        pip install torchvision
        pip install torch
        pip install pyquaternion
        pip install pyyaml
        pip install rospkg
        pip install pexpect
        pip install mujoco==2.3.7
        pip install dm_control==1.0.14
        pip install matplotlib
        pip install einops
        pip install packaging
        pip install h5py
        pip install ipython
        pip install opencv-contrib-python
        pip install ikpy
        pip install pyrealsense2
        pip install csvkit

3. **Install the necessary ROS packages**

        sudo apt install python3-catkin-tools python3-rospkg python3-rosdep python3-rosinstall-generator
        sudo apt install ros-<ros-distro>-geometry-msgs ros-<ros-distro>-sensor-msgs ros-<ros-distro>-nav-msgs
        sudo apt install ros-<ros-distro>-cv-bridge
        sudo apt install ros-<ros-distro>-usb-cam

    Replace `<ros-distro>` with your ROS distribution, e.g., `noetic`.

4. **Intel RealSense Dependencies**  
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

## 2. Workflow

### 2.1 Start ROS Core
Start the ROS master node:

    roscore

### 2.2 Connect RealSense T265 and GoPro
Launch the RealSense and USB camera nodes:

    roslaunch realsense2_camera rs_t265.launch
    roslaunch usb_cam usb_cam-test.launch

### 2.3 Open RViz for Visualization
Launch RViz:

    rviz

- Add **Odometry** and **Image** topics to visualize the data from the RealSense and GoPro cameras.

### 2.4 Run Data Collection Script
1. Update the data saving path in the script.
2. Run the data collection script:

       python data_collection.py

Alternatively, use the provided shell script.

---

## 3. Data Processing Details
### 3.1 Trajectory Transformation
- The raw data consists of the RealSense T265 trajectory.
- An **offset** is added to transform the T265 trajectory into the TCP (Tool Center Point) trajectory.
- Coordinate transformations are applied during this process to map the T265 trajectory to the TCP trajectory.
- The `offset` field in the `config.json` file contains the offsets for the T265 to TCP transformation in the x-axis and z-axis directions.

### 3.2 Gripper Width Detection
- The gripper width is detected using ArUco markers attached to the gripper.
- If any frame fails to detect all the markers, the gripper width is interpolated to ensure complete data.
- The `distances` field in the `config.json` file contains the maximum and minimum pixel distances of the markers in the image (`marker_max` and `marker_min`), as well as the actual maximum gripper width of the robot (`gripper_max`).

### 3.3 Inverse Kinematics (IK) Computation
- A simple IK computation is implemented to convert TCP data into the corresponding absolute joint angles of the robot arm, which facilitates ACT model training.
- To calculate IK, the distance from the TCP to the flange must be obtained. The `distances` field in the `config.json` file includes this distance as `flange_to_tcp`.
