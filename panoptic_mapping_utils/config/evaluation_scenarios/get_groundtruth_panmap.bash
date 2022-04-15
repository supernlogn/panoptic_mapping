#!/bin/bash
# It requires about 20 GB to work... (for large flat dataset)
roslaunch panoptic_mapping_ros runWithVoxgraph.launch\
 use_noise:=True drift_type:=none play_rate:=1 save_pm_trajectory_on_finish:=/home/ioannis/datasets/large_flat/ground_truth_panmap/trajectory.in\
 save_voxgraph_trajectory_on_finish:=/home/ioannis/datasets/large_flat/ground_truth_panmap/voxgraph_traj.bag\
 save_map_path_when_finished:=/home/ioannis/datasets/large_flat/ground_truth_panmap/panmap.panmap\
 config:=flat_groundtruth_single\
 shutdown_when_finished:=True\
 debug_mode:=False
