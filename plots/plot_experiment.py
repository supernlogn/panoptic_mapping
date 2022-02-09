#!/bin/python3
# pylint: skip-file
import plot_trajectories
import os
from matplotlib import pyplot as plt
import yaml

import numpy as np

T_C_R = np.array([[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1]])
home = os.getenv("HOME")
base_dir = os.path.join(home, "datasets")
panoptic_dir = os.path.join(os.path.dirname(__file__), "..")
config_dir = os.path.join(panoptic_dir, "panoptic_mapping_utils", "config")
trajectory_plots = {
    "pm+voxgraph":
    lambda: plot_trajectories.plotPerAxisDirectory(
        base_dir, "temp.png", T_C_R=T_C_R, post_mult=True),
    "voxblox+voxgraph":
    lambda: plot_trajectories.plotPerAxisDirectory(
        base_dir, "temp.png", include_optimized=False),
    "voxblox+voxgraph_rotated":
    lambda: plot_trajectories.plotPerAxisDirectory(base_dir,
                                                   "temp.png",
                                                   include_optimized=False,
                                                   T_C_R=T_C_R,
                                                   post_mult=True),
    "pm+voxgraph_input":
    lambda: plot_trajectories.plotFiles(
        {
            'ground_truth_tr': os.path.join(base_dir, "generated_path.txt"),
            'optimized_tr': os.path.join(base_dir, "trajectory.in"),
            "voxgraph_tr": os.path.join(base_dir, "vox_input.bag")
        },
        base_dir,
        "temp",
        T_C_R=T_C_R,
        post_mult=True),
    "experiments_w_wo_alignment":
    lambda: plot_trajectories.plotFullExperiment(
        os.path.join(config_dir,
                     'evaluation_experiments/experiments_w_wo_alignment.yaml')
    ),
    "experiments_w_wo_interpolation":
    lambda: plot_trajectories.plotFullExperiment(
        os.path.join(
            config_dir,
            'evaluation_experiments/experiments_w_wo_interpolation.yaml'))
}


def main():
    trajectory_plots["pm+voxgraph"]()


if __name__ == "__main__":
    main()