#!/bin/python3
# pylint: skip-file
import plot_trajectories
import os
from matplotlib import pyplot as plt
import yaml

import numpy as np


def main():
    # plotFullExperiment('/home/ioannis/catkin_ws/src/panoptic_mapping/panoptic_mapping_utils/config/evaluation_experiments/experiments_w_wo_interpolation.yaml')
    # plotFullExperiment(
    #     '/home/ioannis/catkin_ws/src/panoptic_mapping/panoptic_mapping_utils/config/evaluation_experiments/experiments_w_wo_alignment.yaml'
    # )
    # plotPerAxisDirectory("/home/ioannis/datasets/z_fix", "z_fix.png")
    T_C_R = (np.array([[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0],
                       [0, 0, 0, 1]]))
    # plot_trajectories.plotPerAxisDirectory("/home/ioannis/datasets", "temp.png")#, T_C_R=T_C_R)
    plot_trajectories.plotPerAxisDirectory(
        "/home/ioannis/datasets", "temp.png",
        include_optimized=False)  #, T_C_R=T_C_R)


if __name__ == "__main__":
    main()