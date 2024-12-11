import os
import h5py
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

def load_hdf5(dataset_path):
    if not os.path.isfile(dataset_path):
        print(f'Dataset does not exist at {dataset_path}')
        exit()

    with h5py.File(dataset_path, 'r') as root:
        qpos = root['/observations/qpos'][()]  # Get joint positions
        # Assuming the structure of qpos is: Pos X, Pos Y, Pos Z, Q_X, Q_Y, Q_Z, Q_W
        return qpos

def plot_rotations(qpos):
    # Extract quaternion data (Q_X, Q_Y, Q_Z, Q_W)
    quaternions = qpos[:, 3:]  # Assuming the last four columns are quaternions (Q_X, Q_Y, Q_Z, Q_W)

    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    for i in range(len(quaternions)):
        q = quaternions[i]
        r = R.from_quat(q)  # Create a rotation object from the quaternion
        rot_matrix = r.as_matrix()  # Get the rotation matrix

        # Compute the rotated direction vector, here using [1, 0, 0] as an example
        direction = rot_matrix @ np.array([1, 0, 0])  # The rotated direction vector
        
        # Visualize the rotation direction vector, adjusting the length parameter to control the arrow size
        ax.quiver(qpos[i, 0], qpos[i, 1], qpos[i, 2], 
                  direction[0], direction[1], direction[2], 
                  length=0.03, color='r', normalize=True)  # length=0.03 makes the arrows shorter, normalize=True normalizes the vector

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Rotation Visualization')

    plt.show()


# Load the data
dataset_path = 'path to your hdf5'  # Change this to the actual path
qpos = load_hdf5(dataset_path)

# Visualize the trajectory
plot_rotations(qpos)
