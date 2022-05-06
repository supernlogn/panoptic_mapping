#!/bin/bash
# Evaluation of all small flat experiments
# that depend on time.
# This batch file should be executed after
# running:
# /panoptic_mapping_utils/src/evaluation/run_evaluations.py panoptic_mapping_utils/config/evaluation_scenarios/small_flat/temporal_voxgraph_map_metric.yaml
#1
gt_file=/home/ioannis/datasets/flat_dataset/flat_2_gt_10000.ply
base_dir=/home/ioannis/datasets/temporal_voxgraph_map_metric
A=$(ls -d ${base_dir}/experiment1/*/)
roslaunch panoptic_mapping_utils evaluate_series.launch ground_truth_pointcloud_file:=$gt_file output_suffix:=evaluations map_file:=$A is_single_tsdf:=True
#2
A=$(ls -d ${base_dir}/experiment2/*/)
roslaunch panoptic_mapping_utils evaluate_series.launch ground_truth_pointcloud_file:=$gt_file output_suffix:=evaluations map_file:=$A is_single_tsdf:=True
#4
A=$(ls -d ${base_dir}/experiment4/*/)
roslaunch panoptic_mapping_utils evaluate_series.launch ground_truth_pointcloud_file:=$gt_file output_suffix:=evaluations map_file:=$A
#5
A=$(ls -d ${base_dir}/experiment5/*/)
roslaunch panoptic_mapping_utils evaluate_series.launch ground_truth_pointcloud_file:=$gt_file output_suffix:=evaluations map_file:=$A
