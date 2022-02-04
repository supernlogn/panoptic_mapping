# compare inputs that voxgraph gets from:
# - panoptic mapping
# - voxblox
import os
import math

from matplotlib import pyplot as plt
import plot_trajectories

PI = math.pi
home = os.getenv("HOME")
pm_trajectory, pm_times = plot_trajectories.getTrajectoryFromBagFile(
    os.path.join(home, 'voxgraph_input_pm.bag'))
voxblox_trajectory, voxblox_times = plot_trajectories.getTrajectoryFromBagFile(
    os.path.join(home, 'voxgraph_input_voxblox.bag'))

index = 5
plt.plot(pm_times, pm_trajectory[:, index], color='red')
plt.plot(voxblox_times, voxblox_trajectory[:, index], color='blue')
plt.show()
