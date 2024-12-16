import os
import re
import numpy as np
from scipy.spatial.transform import Rotation
import h5py
import json
import click
import zarr
import numpy as np
import cv2
import multiprocessing
from replay_buffer import ReplayBuffer
from imagecodecs_numcodecs import register_codecs, JpegXl
register_codecs()

# Load the configuration from the config.json file
with open('config/config.json', 'r') as config_file:
    config = json.load(config_file)
config = config["data_process_config"]

def mat_to_pos_rot(mat):
    pos = (mat[...,:3,3].T / mat[...,3,3].T).T
    rot = Rotation.from_matrix(mat[...,:3,:3])
    return pos, rot

def pos_rot_to_pose(pos, rot):
    shape = pos.shape[:-1]
    pose = np.zeros(shape+(6,), dtype=pos.dtype)
    pose[...,:3] = pos
    pose[...,3:] = rot.as_rotvec()
    return pose

def mat_to_pose(mat):
    return pos_rot_to_pose(*mat_to_pos_rot(mat))

def get_image_transform(in_res, out_res, crop_ratio:float = 1.0, bgr_to_rgb: bool=False):
    iw, ih = in_res
    ow, oh = out_res
    ch = round(ih * crop_ratio)
    cw = round(ih * crop_ratio / oh * ow)
    interp_method = cv2.INTER_AREA

    w_slice_start = (iw - cw) // 2
    w_slice = slice(w_slice_start, w_slice_start + cw)
    h_slice_start = (ih - ch) // 2
    h_slice = slice(h_slice_start, h_slice_start + ch)
    c_slice = slice(None)
    if bgr_to_rgb:
        c_slice = slice(None, None, -1)

    def transform(img: np.ndarray):
        assert img.shape == ((ih,iw,3))
        # crop
        img = img[h_slice, w_slice, c_slice]
        # resize
        img = cv2.resize(img, out_res, interpolation=interp_method)
        return img
    
    return transform

def main():

    out_res = config['dp_data_res']
    # Retrieve all HDF5 files in the folder and sort them in ascending order based on the numbers in their filenames
    hdf5_files = sorted(
        [f for f in os.listdir(input_folder_dir) if f.endswith('.hdf5')],
        key=lambda x: int(''.join(filter(str.isdigit, x)))  
    )

    for _, hdf5_file in enumerate(hdf5_files):
        hdf5_path = os.path.join(input_folder_dir, hdf5_file)
        with h5py.File(hdf5_path, 'r') as f:
            action_data = f['action']
            pos = np.array(action_data[:,:3])
            rot_quat_xyzw = np.array(action_data[:, 3:7])
            gripper_width = np.array(action_data[:, 7])

        rot = Rotation.from_quat(rot_quat_xyzw)
        pose = np.zeros((pos.shape[0], 4, 4), dtype=np.float32)
        pose[:,3,3] = 1
        pose[:,:3,3] = pos
        pose[:,:3,:3] = rot.as_matrix()
        pose = mat_to_pose(pose)

        demo_start_pose = pose[0]
        demo_end_pose = pose[-1]

        grippers_dict = {
            'tcp_pose': np.array(pose),
            'gripper_width': np.array(gripper_width),
            'demo_start_pose': np.array(demo_start_pose),
            'demo_end_pose': np.array(demo_end_pose)
        }
        data_dict = {
            'grippers': [grippers_dict]
        }
        data_list.append(data_dict)

    if os.path.isfile(output_dir):
        if click.confirm(f'Output file {output_dir} exists! Overwrite?', abort=True):
            pass
        
    out_res = tuple(int(x) for x in out_res.split(','))
    num_workers = None
    if num_workers is None:
        num_workers = multiprocessing.cpu_count()
    cv2.setNumThreads(1)
        
    out_replay_buffer = ReplayBuffer.create_empty_zarr(
        storage=zarr.MemoryStore())
    
    n_grippers = None
    n_cameras = 1
        
    for plan_episode in data_list:
        grippers = plan_episode['grippers']
        
        # check that all episodes have the same number of grippers 
        if n_grippers is None:
            n_grippers = len(grippers)
        else:
            assert n_grippers == len(grippers)
                        
        episode_data = dict()
        for gripper_id, gripper in enumerate(grippers):
                
            eef_pose = gripper['tcp_pose']
            eef_pos = eef_pose[...,:3]
            eef_rot = eef_pose[...,3:]
            gripper_widths = gripper['gripper_width']
            demo_start_pose = np.empty_like(eef_pose)
            demo_start_pose[:] = gripper['demo_start_pose']
            demo_end_pose = np.empty_like(eef_pose)
            demo_end_pose[:] = gripper['demo_end_pose']
            
            robot_name = f'robot{gripper_id}'
            episode_data[robot_name + '_eef_pos'] = eef_pos.astype(np.float32)
            episode_data[robot_name + '_eef_rot_axis_angle'] = eef_rot.astype(np.float32)
            episode_data[robot_name + '_gripper_width'] = np.expand_dims(gripper_widths, axis=-1).astype(np.float32)
            episode_data[robot_name + '_demo_start_pose'] = demo_start_pose
            episode_data[robot_name + '_demo_end_pose'] = demo_end_pose
        
        out_replay_buffer.add_episode(data=episode_data, compressors=None)

    resize_tf = get_image_transform(
            in_res=(1920, 1080),
            out_res=out_res
        )
    
    # dump images
    img_compressor = JpegXl(level=compression_level, numthreads=1)
    for cam_id in range(n_cameras):
        name = f'camera{cam_id}_rgb'
        _ = out_replay_buffer.data.require_dataset(
            name=name,
            shape=(out_replay_buffer['robot0_eef_pos'].shape[0],) + out_res + (3,),
            chunks=(1,) + out_res + (3,),
            compressor=img_compressor,
            dtype=np.uint8
        )

    buffer_index = 0
    directory = config['input_dir']
    files= [f for f in os.listdir(directory) if f.endswith('.hdf5')]
    files.sort(key=lambda f: int(re.search(r'episode_(\d+)', f).group(1)))
    for filename in files:
        input_file = os.path.join(directory, filename)
        print(f"Processing {input_file}")
        with h5py.File(input_file,'r') as f:
            name = 'camera0_rgb'
            img_array = out_replay_buffer.data[name]
            img = np.array(f['observations/images/front'])
            for i in range(img.shape[0]):
                resize_img = resize_tf(img[i])
                img_array[buffer_index] = resize_img
                buffer_index += 1

    print(f"Saving ReplayBuffer to {output_dir}")
    with zarr.ZipStore(output_dir, mode='w') as zip_store:
        out_replay_buffer.save_to_store(
            store=zip_store
        )
 
if __name__ == "__main__":

    input_folder_dir = config['output_tcp_dir']
    output_dir = config['dp_train_data_dir']
    compression_level = config['compression_level']
    data_list = []

    main()