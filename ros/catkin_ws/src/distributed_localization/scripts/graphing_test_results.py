# This import registers the 3D projection, but is otherwise unused.
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

import matplotlib.pyplot as plt
import numpy as np
import glob

def get_data(test, robot):
    """
    Returns time, amcl_pose, actual_pose
    """
    # Load in amcl data
    data = np.loadtxt(f'testing_data/test-{test}-{robot}-amcl.txt')
    amcl_time = data[:, 0]
    amcl_pose = data[:, 1:]
    # Load in actual data
    data = np.loadtxt(f'testing_data/test-{test}-{robot}-actual.txt')
    actual_pose = data[:, 1:]

    return amcl_time, amcl_pose, actual_pose

if __name__ == "__main__":
    # Get average error per robot per time step
    error_sum_local = 0
    num_time_local = 0

    for test in range(1, 101, 2):
        for robot, marker in zip(['tb3_1', 'tb3_2', 'tb3_3'], ['1', '2', '3']):
            time, amcl_pose, actual_pose = get_data(test, robot)
            num_time_local += len(time)
            error_sum_local += np.sum(np.linalg.norm(amcl_pose[:, 0:2] - actual_pose[:, 0:2], axis=-1))
    
    error_sum_dist = 0
    num_time_dist = 0

    for test in range(2, 101, 2):
        for robot, marker in zip(['tb3_1', 'tb3_2', 'tb3_3'], ['1', '2', '3']):
            time, amcl_pose, actual_pose = get_data(test, robot)
            num_time_dist += len(time)
            error_sum_local += np.sum(np.linalg.norm(amcl_pose[:, 0:2] - actual_pose[:, 0:2], axis=-1))
    
    print(f'Local Error: {error_sum_local/(num_time_local)}')
    print(f'Distributed Error: {error_sum_local/(num_time_dist)}')



    

    fig = plt.figure()
    ax = fig.add_subplot(111)
    
    for test in range(1, 101):
        for robot, marker in zip(['tb3_1', 'tb3_2', 'tb3_3'], ['1', '2', '3']):
            time, amcl_pose, actual_pose = get_data(test, robot)
            plt.scatter(time, np.linalg.norm(amcl_pose[:, 0:2] - actual_pose[:, 0:2], axis=-1), s=2, marker=marker, label=robot)
            
        ax.legend()
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Error in x,y position estimate (m)')
        plt.title(f"Experiment #{test} {'Distributed' if test%2==0 else 'Local'}")

        plt.show()