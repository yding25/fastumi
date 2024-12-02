"""
Robot Data Collection and Recording System

This script implements a data collection system for robotics tasks, recording video, trajectory,
and state information. It uses ROS for communication and supports multiple episodes of recording.
"""

import os
import cv2
import h5py
import rospy
import numpy as np
import pandas as pd
import argparse
import threading
import csv
from tqdm import tqdm
from time import sleep
from collections import deque
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

from config.config import TASK_CONFIG, ROBOT_TYPE

class DataRecorder:
    def __init__(self, task_name, num_episodes):
        """Initialize the data recorder with task configuration and paths."""
        self.cfg = TASK_CONFIG
        self.robot = ROBOT_TYPE
        self.task = task_name
        self.num_episodes = num_episodes
        
        # Initialize data paths
        self.setup_data_paths()
        
        # Initialize ROS node and parameters
        rospy.init_node('video_trajectory_recorder', anonymous=True)
        self.cv_bridge = CvBridge()
        
        # Recording parameters
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.frame_width, self.frame_height = 1920, 1080
        
        # Buffers for data synchronization
        self.video_buffer = deque()
        self.trajectory_buffer = deque()
        self.buffer_lock = threading.Lock()
        
        # Initialize subscribers
        self.video_subscriber = None
        self.trajectory_subscriber = None
        self.first_frame_timestamp = None
        self.start_time = 0
        self.first_time_judger = False

    def setup_data_paths(self):
        """Set up necessary directories and paths for data storage."""
        self.data_path = os.path.join('/home/zhaxizhuoma/data/ACT/', str(self.task))
        self.image_path = os.path.join(self.data_path, 'camera/')
        self.csv_path = os.path.join(self.data_path, 'csv/')
        self.state_path = os.path.join(self.data_path, 'states.csv')
        
        # Create directories if they don't exist
        for path in [self.data_path, self.image_path, self.csv_path]:
            os.makedirs(path, exist_ok=True)
            
        # Initialize state file if it doesn't exist
        if not os.path.exists(self.state_path):
            with open(self.state_path, 'w') as f:
                pass

    def video_callback(self, msg):
        """Process incoming video frames from ROS topic."""
        frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        timestamp = msg.header.stamp.to_sec()
        
        with self.buffer_lock:
            if self.start_time < timestamp:
                self.video_buffer.append((frame, timestamp))
                if self.first_time_judger:
                    self.first_frame_timestamp = timestamp
                    self.first_time_judger = False

    def trajectory_callback(self, msg):
        """Process incoming trajectory data from ROS topic."""
        timestamp = msg.header.stamp.to_sec()
        with self.buffer_lock:
            if self.start_time < timestamp:
                pose = msg.pose.pose
                self.trajectory_buffer.append((
                    timestamp,
                    pose.position.x, pose.position.y, pose.position.z,
                    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
                ))

    def write_video(self):
        """Write video frames to file with progress tracking."""
        frame_index = 0
        previous_progress = 0
        pbar = tqdm(total=self.cfg['episode_len'], desc='Processing Frames')

        while not rospy.is_shutdown():
            with self.buffer_lock:
                if self.video_buffer:
                    frame, timestamp = self.video_buffer.popleft()
                    self.video_writer.write(frame)
                    self.timestamp_writer.writerow([frame_index, timestamp])
                    
                    frame_index += 1
                    current_progress = frame_index // 3
                    pbar.update(current_progress - previous_progress)
                    previous_progress = current_progress
                    
                    if frame_index == 3 * self.cfg['episode_len']:
                        print("Video recording completed")
                        pbar.close()
                        break
            sleep(0.001)

    def write_trajectory(self):
        """Write trajectory data to CSV file."""
        counter = 0
        while not rospy.is_shutdown():
            with self.buffer_lock:
                if self.trajectory_buffer:
                    data = self.trajectory_buffer.popleft()
                    self.trajectory_writer.writerows([data])
                    counter += 1
                    if counter == 10 * self.cfg['episode_len']:
                        print("Trajectory recording completed")
                        break
            sleep(0.001)

    def start_recording(self):
        """Start recording video and trajectory data."""
        self.video_buffer.clear()
        self.trajectory_buffer.clear()

        video_thread = threading.Thread(target=self.write_video)
        trajectory_thread = threading.Thread(target=self.write_trajectory)

        video_thread.start()
        trajectory_thread.start()
        video_thread.join()
        trajectory_thread.join()

    def process_episode(self, episode_idx):
        """Process and save a single episode of data collection."""
        temp_paths = {
            'video': os.path.join(self.data_path, f'camera/temp_video_{episode_idx}.mp4'),
            'trajectory': os.path.join(self.data_path, 'csv/temp_trajectory.csv'),
            'timestamp': os.path.join(self.data_path, 'csv/temp_video_timestamps.csv')
        }

        # Initialize writers and files
        self.video_writer = cv2.VideoWriter(temp_paths['video'], self.fourcc, 60, (self.frame_width, self.frame_height))
        self.trajectory_file = open(temp_paths['trajectory'], 'w')
        self.timestamp_file = open(temp_paths['timestamp'], 'w')
        
        self.trajectory_writer = csv.writer(self.trajectory_file)
        self.timestamp_writer = csv.writer(self.timestamp_file)
        
        # Write headers
        self.trajectory_writer.writerow(['Timestamp', 'Pos X', 'Pos Y', 'Pos Z', 'Q_X', 'Q_Y', 'Q_Z', 'Q_W'])
        self.timestamp_writer.writerow(['Frame Index', 'Timestamp'])

        # Start recording
        input(f"Press Enter to start episode {episode_idx + 1}/{self.num_episodes}")
        self.start_time = rospy.Time.now().to_sec()
        self.first_time_judger = True
        print(f"Recording episode {episode_idx + 1}/{self.num_episodes}")

        try:
            self.start_recording()
        finally:
            self.cleanup_episode(episode_idx, temp_paths)

    def cleanup_episode(self, episode_idx, temp_paths):
        """Clean up and save episode data."""
        self.video_writer.release()
        self.trajectory_file.close()
        self.timestamp_file.close()

        # Process and save the collected data
        self.save_episode_data(episode_idx, temp_paths)

    def save_episode_data(self, episode_idx, temp_paths):
        """Save episode data to HDF5 format."""
        data_dict = {
            '/observations/qpos': [],
            '/action': [],
        }
        for cam_name in self.cfg['camera_names']:
            data_dict[f'/observations/images/{cam_name}'] = []

        # Process video frames
        timestamps = pd.read_csv(temp_paths['timestamp'])
        downsampled_timestamps = timestamps.iloc[::3].reset_index(drop=True)
        
        # Save frames
        with cv2.VideoCapture(temp_paths['video']) as cap:
            for i, row in tqdm(downsampled_timestamps.iterrows(), desc='Processing images'):
                cap.set(cv2.CAP_PROP_POS_FRAMES, row['Frame Index'])
                ret, frame = cap.read()
                filename = f"{int(row['Frame Index'] / 3)}.jpg"
                cv2.imwrite(os.path.join(self.image_path, filename), frame)
                
                for cam_name in self.cfg['camera_names']:
                    data_dict[f'/observations/images/{cam_name}'].append(frame)

        # Process trajectory data
        trajectory = pd.read_csv(temp_paths['trajectory'])
        trajectory['Timestamp'] = trajectory['Timestamp'].astype(float)

        # Save to HDF5
        self.save_to_hdf5(data_dict, episode_idx, downsampled_timestamps, trajectory)

    def save_to_hdf5(self, data_dict, episode_idx, timestamps, trajectory):
        """Save processed data to HDF5 file format."""
        data_dir = os.path.join(self.cfg['dataset_dir'], self.task)
        os.makedirs(data_dir, exist_ok=True)
        
        idx = len([name for name in os.listdir(data_dir) if os.path.isfile(os.path.join(data_dir, name))])
        dataset_path = os.path.join(data_dir, f'episode_{idx}.hdf5')
        
        with h5py.File(dataset_path, 'w', rdcc_nbytes=1024 ** 2 * 2) as root:
            root.attrs['sim'] = True
            obs = root.create_group('observations')
            image = obs.create_group('images')
            
            for cam_name in self.cfg['camera_names']:
                image.create_dataset(
                    cam_name,
                    (len(data_dict['/observations/qpos']), self.cfg['cam_height'], 
                        self.cfg['cam_width'], 3),
                    dtype='uint8',
                    chunks=(1, self.cfg['cam_height'], self.cfg['cam_width'], 3),
                    compression='gzip',
                    compression_opts=4
                )
            
            obs.create_dataset('qpos', (len(data_dict['/observations/qpos']), self.cfg['state_dim']))
            root.create_dataset('action', (len(data_dict['/observations/qpos']), self.cfg['action_dim']))
            
            for name, array in data_dict.items():
                root[name][...] = array

def main():
    """Main function to run the data collection system."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--task', type=str, default="test", 
                        help="Task name (e.g., open_lid, open_fridge, open_drawer, pick_place_pot)")
    parser.add_argument('--num_episodes', type=int, default=2,
                        help="Number of episodes to record")
    args = parser.parse_args()

    recorder = DataRecorder(args.task, args.num_episodes)
    
    # Set up ROS subscribers
    recorder.video_subscriber = rospy.Subscriber(
        "/usb_cam/image_raw", Image, recorder.video_callback, queue_size=1000)
    recorder.trajectory_subscriber = rospy.Subscriber(
        "/camera/odom/sample", Odometry, recorder.trajectory_callback, queue_size=1000)

    # Record episodes
    for i in range(args.num_episodes):
        recorder.process_episode(i)

if __name__ == "__main__":
    main()