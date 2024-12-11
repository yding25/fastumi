import os
import h5py
import cv2

def load_hdf5(dataset_path):
    # Check if the file exists
    if not os.path.isfile(dataset_path):
        print(f'Dataset does not exist at \n{dataset_path}\n')
        exit()

    with h5py.File(dataset_path, 'r') as root:
        qpos = root['/observations/qpos'][()]  # Get joint positions
        image_dict = dict()  # Dictionary to store images from different cameras
        for cam_name in root[f'/observations/images/'].keys():
            image_dict[cam_name] = root[f'/observations/images/{cam_name}'][()]
        action = root['/action'][()]  # Get action information

    return qpos, action, image_dict

def write_video_cv2(image_list, output_file, fps=60):
    # Get the height and width of the images
    height, width, _ = image_list[0].shape
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Use the 'mp4v' codec
    out = cv2.VideoWriter(output_file, fourcc, fps, (width, height))

    # Write the video frames (no color conversion)
    for image in image_list:
        out.write(image)  # Write the original image directly

    out.release()  # Release resources

def save_videos(image_dict, save_dir, task, hdf5_file_name, fps=60):
    # Create the save directory if it doesn't exist
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    
    # Save video for each camera
    for cam_name, image_list in image_dict.items():
        print(f"Saving video from camera: {cam_name}")
        # Generate a unique output file path using task + hdf5 filename + camera name
        output_video_file = os.path.join(save_dir, f'{task}_{hdf5_file_name}_{cam_name}.mp4')
        write_video_cv2(image_list, output_video_file, fps)
        print(f"Video saved to {output_video_file}")

def process_hdf5_files_in_directory(directory_path, save_dir, task, fps=30):
    # Get all .hdf5 files in the specified directory
    hdf5_files = [f for f in os.listdir(directory_path) if f.endswith('.hdf5')]

    if len(hdf5_files) == 0:
        print("No .hdf5 files found in the specified directory.")
        return

    # Process each hdf5 file
    for hdf5_file in hdf5_files:
        print(f"Processing file: {hdf5_file}")
        file_path = os.path.join(directory_path, hdf5_file)

        # Load data from the hdf5 file
        qpos, action, image_dict = load_hdf5(file_path)

        # Extract the file name without the extension for naming the video
        hdf5_file_name = os.path.splitext(hdf5_file)[0]

        # Save videos
        save_videos(image_dict, save_dir, task, hdf5_file_name, fps)

# Define input and output directories
input_directory = 'path to your dataset folder'  # Change this to the folder containing your .hdf5 files
output_directory = 'path to your output folder'  # Change this to the folder where you want to save the videos
task = "your task"  # Task name, change this based on your requirements

# Run the batch processing
process_hdf5_files_in_directory(input_directory, output_directory, task, fps=20)
