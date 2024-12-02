from config.config import TASK_CONFIG, ROBOT_TYPE
import os
import cv2
import h5py
import argparse
from tqdm import tqdm
from time import sleep, time
from model.utils import pwm2pos, pwm2vel
import numpy as np
# from robot import Robot
# from xarm.wrapper import XArmAPI
import pyrealsense2 as rs
import apriltag
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
import csv
from scipy.spatial.transform import Rotation as R

import threading
from collections import deque
from datetime import datetime
import pandas as pd
# INITAL_POSE = [0.4964, 0.0050, 0.3598]


# parse the task name via command line
parser = argparse.ArgumentParser()
parser.add_argument('--task', type=str, default="test") # open_lid, open_fridge, open_drawer, pick_place_pot
parser.add_argument('--num_episodes', type=int, default=2)
args = parser.parse_args()
task = args.task
num_episodes = args.num_episodes


cfg = TASK_CONFIG
robot = ROBOT_TYPE

DATA_PATH = '/home/zhaxizhuoma/data/ACT/'

data_path = os.path.join(DATA_PATH, str(task))
if not os.path.exists(data_path):
    os.makedirs(data_path)

IMAGE_PATH = os.path.join(data_path, 'camera/')
if not os.path.exists(IMAGE_PATH):
    os.makedirs(IMAGE_PATH)

CSV_PATH = os.path.join(data_path, 'csv/')
if not os.path.exists(CSV_PATH):
    os.makedirs(CSV_PATH)

STATE_PATH = os.path.join(data_path, 'states.csv')
if not os.path.exists(STATE_PATH):
    csv_file2 = open(STATE_PATH, 'w')
    csv_writer2 = csv.writer(csv_file2)
    csv_file2.close()

VIDEO_PATH_TEMP = os.path.join(DATA_PATH, str(task)+'/camera/temp_video_n.mp4')
TRAJECTORY_PATH_TEMP = os.path.join(DATA_PATH, str(task)+'/csv/temp_trajectory.csv')
TIMESTAMP_PATH_TEMP = os.path.join(DATA_PATH, str(task)+'/csv/temp_video_timestamps.csv')
FRAME_TIMESTAMP_PATH_TEMP = os.path.join(DATA_PATH, str(task)+'/csv/frame_timestamps.csv')

video_subscriber = None
trajectory_subscriber = None

# Initialize ROS node
rospy.init_node('video_trajectory_recorder', anonymous=True)

# Video writer parameters for 60 Hz recording
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
frame_width, frame_height = 1920, 1080

# Buffers for storing incoming data
video_buffer = deque()
trajectory_buffer = deque()

# Lock for thread synchronization
buffer_lock = threading.Lock()

# Initialize CvBridge for image conversion
cv_bridge = CvBridge()

# Variable to store the first frame's timestamp
first_frame_timestamp = None


# Callback for video frames (60 Hz expected)
def video_callback(msg):
    global first_frame_timestamp, first_time_judger
    frame = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    timestamp = msg.header.stamp.to_sec()

    # Capture the first frame timestamp
    # if first_frame_timestamp is None:
    #     first_frame_timestamp = timestamp

    # Append the frame and timestamp to the video buffer
    with buffer_lock:
        if start_time < timestamp:
            video_buffer.append((frame, timestamp))
            if first_time_judger:
                first_frame_timestamp = timestamp
                first_time_judger = False
            

# Callback for trajectory data (e.g., T265 at 200 Hz)
def trajectory_callback(msg):
    timestamp = msg.header.stamp.to_sec() # Ensure timestamp is in Unix format (float)
    with buffer_lock:
        if start_time < timestamp:
            pose = msg.pose.pose
            trajectory_buffer.append((timestamp, pose.position.x, pose.position.y, pose.position.z,
                                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))

# Thread for writing video frames and timestamps
def write_video():
    frame_index = 0
    previous_progress = 0  # Keep track of the last progress value
    pbar = tqdm(total=cfg['episode_len'], desc='Processing Frames')

    while not rospy.is_shutdown():
        with buffer_lock:
            if video_buffer:
                frame, timestamp = video_buffer.popleft()
                video_writer.write(frame)
                
                # Write timestamp for each frame to CSV
                timestamp_writer.writerow([frame_index, timestamp])
                frame_index += 1
                
                # Update progress bar
                current_progress = frame_index // 3
                pbar.update(current_progress - previous_progress)
                previous_progress = current_progress
                
                if frame_index == 3 * cfg['episode_len']:
                    print("Video Done!")
                    pbar.close()  # Close the progress bar
                    break

        sleep(0.001)  # Small sleep to avoid CPU overload

# Thread for writing trajectory data to CSV
def write_trajectory():
    counter = 0
    while not rospy.is_shutdown():
        with buffer_lock:
            if trajectory_buffer:
                Timestamp, PosX, PosY, PosZ,  Q_X, Q_Y, Q_Z, Q_W = trajectory_buffer.popleft()
                trajectory_writer.writerows([(Timestamp, PosX, PosY, PosZ,  Q_X, Q_Y, Q_Z, Q_W)])
                # trajectory_buffer.clear()
                counter += 1
                # print(counter)
                if counter == 10 * cfg['episode_len']:
                    print("Trajectory Done!")
                    break
            
        sleep(0.001)  # Small sleep to avoid CPU overload

# Main function to start recording
def start_recording():
    global video_subscriber, trajectory_subscriber
    # Subscribe to video and trajectory topics

    video_buffer.clear()
    trajectory_buffer.clear()

    # Start separate threads for writing video, timestamps, and trajectory data
    video_thread = threading.Thread(target=write_video)
    trajectory_thread = threading.Thread(target=write_trajectory)

    video_thread.start()
    trajectory_thread.start()

    video_thread.join()
    trajectory_thread.join()


if __name__ == "__main__":


    # init
    start_time = 0
    cv_bridge = CvBridge()
    video_subscriber = rospy.Subscriber("/usb_cam/image_raw", Image, video_callback, queue_size=1000)
    trajectory_subscriber = rospy.Subscriber("/camera/odom/sample", Odometry, trajectory_callback, queue_size=1000)
    # ! Teleoperation
    frame_timestamp_file = open(FRAME_TIMESTAMP_PATH_TEMP, "a")
    frame_timestamp_writer = csv.writer(frame_timestamp_file)
    frame_timestamp_writer.writerow(['episodes Index', 'Timestamp'])
    for i in range(num_episodes):
        video_writer = cv2.VideoWriter(VIDEO_PATH_TEMP.replace("_n", "_"+str(i)), fourcc, 60, (frame_width, frame_height))

        # CSV for trajectory data and video timestamps
        trajectory_file = open(TRAJECTORY_PATH_TEMP, 'w')
        timestamp_file = open(TIMESTAMP_PATH_TEMP, 'w')

        trajectory_writer = csv.writer(trajectory_file)
        timestamp_writer = csv.writer(timestamp_file)

        # Write headers to CSV files
        trajectory_writer.writerow(['Timestamp', 'Pos X', 'Pos Y', 'Pos Z', 'Q_X', 'Q_Y', 'Q_Z', 'Q_W'])
        timestamp_writer.writerow(['Frame Index', 'Timestamp'])
        
        first_time_judger = False


        input(f"episode {i+1}/{num_episodes} wait")
        # sleep(0.5)
        start_time = rospy.Time.now().to_sec() # ! start time
        first_time_judger = True
        # TODO START
        print(f"episode {i+1}/{num_episodes} GO!")

        # init buffers
        obs_replay = []
        action_replay = []

        frame_timestamp_file = open(FRAME_TIMESTAMP_PATH_TEMP, "a")
        frame_timestamp_writer = csv.writer(frame_timestamp_file)
        #start recording
        try:
            start_recording()
        except Exception as e:
            print(f"An error occurred: {e}")
            raise  # 重新抛出异常，终止代码执行
        finally:
            frame_timestamp_writer.writerow([i] + [first_frame_timestamp])
            video_writer.release()
            trajectory_file.close()
            timestamp_file.close()
            #data list prepare
            data_dict = {
                '/observations/qpos': [],
                '/action': [],
            }
            for cam_name in cfg['camera_names']:
                    data_dict[f'/observations/images/{cam_name}'] = []

            timestamps = pd.read_csv(TIMESTAMP_PATH_TEMP)
            downsampled_timestamps = timestamps.iloc[::3].reset_index(drop=True)
            cap = cv2.VideoCapture(VIDEO_PATH_TEMP.replace("_n", "_"+str(i)))
            #get downsampled image
            for i, row in tqdm(downsampled_timestamps.iterrows(),desc='get images'):
                frame_idx = row['Frame Index']
                # print(frame_idx)
                
                cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
                ret, frame = cap.read()
                # print(frame)
                filename = str(int(frame_idx / 3)) + '.jpg'
                cv2.imwrite(str(IMAGE_PATH) + filename, frame)
                for cam_name in cfg['camera_names']:
                    data_dict[f'/observations/images/{cam_name}'].append(frame)

            cap.release()
            trajectory = pd.read_csv(TRAJECTORY_PATH_TEMP)
            trajectory['Timestamp'] = trajectory['Timestamp'].astype(float)
            
            timestamp_list = []
            for i, row in tqdm(downsampled_timestamps.iterrows(),desc='get states'):
                # Calculate timestamp as first_video_timestamp + 0.05 * i            
                # Find the closest trajectory data point
                closest_idx = (np.abs(trajectory['Timestamp'] - row['Timestamp'])).argmin()
                closest_row = trajectory.iloc[closest_idx]
                _pos_and_quat = [closest_row['Pos X'], closest_row['Pos Y'], closest_row['Pos Z'], closest_row['Q_X'], closest_row['Q_Y'], closest_row['Q_Z'], closest_row['Q_W']]
                data_dict['/observations/qpos'].append(_pos_and_quat)
                data_dict['/action'].append(_pos_and_quat)
                csv_file2 = open(STATE_PATH, 'a') # 'a' means append row into file
                csv_writer2 = csv.writer(csv_file2)
                csv_writer2.writerow([i] + [start_time] + [closest_row['Timestamp']] + [row['Timestamp']] + _pos_and_quat)
            
            frame_timestamp_file.close()

            
            max_timesteps = len(data_dict['/observations/qpos']) # ! discrete k, not continuous time
            # create data dir if it doesn't exist
            data_dir = os.path.join(cfg['dataset_dir'], task)
            if not os.path.exists(data_dir): os.makedirs(data_dir)
            # count number of files in the directory
            idx = len([name for name in os.listdir(data_dir) if os.path.isfile(os.path.join(data_dir, name))])
            dataset_path = os.path.join(data_dir, f'episode_{idx}')
            print(dataset_path)
            # save the data
            with h5py.File(dataset_path + '.hdf5', 'w', rdcc_nbytes=1024 ** 2 * 2) as root:
                root.attrs['sim'] = True
                obs = root.create_group('observations')
                image = obs.create_group('images')
                for cam_name in cfg['camera_names']:
                    _ = image.create_dataset(
                                                cam_name,
                                                (max_timesteps, cfg['cam_height'], cfg['cam_width'], 3),
                                                dtype='uint8',
                                                chunks=(1, cfg['cam_height'], cfg['cam_width'], 3),
                                                compression='gzip',  # 使用 'gzip' 压缩
                                                compression_opts=4   # 压缩级别，0（最快）到9（最高压缩）
                                            )
                qpos = obs.create_dataset('qpos', (max_timesteps, cfg['state_dim']))
                action = root.create_dataset('action', (max_timesteps, cfg['action_dim']))
                for name, array in data_dict.items():
                    root[name][...] = array
            # os.remove(VIDEO_PATH_TEMP)
            # os.remove(TRAJECTORY_PATH_TEMP)
            # os.remove(TIMESTAMP_PATH_TEMP)
            csv_file2.close()
