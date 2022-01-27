#!/bin/bash



function killRos () {
    pkill -9 rosmaster;
    pkill -9 roscore;
}
function callVoxgraph() {
    num_Args=$#
    args="play_rate:=5 use_noise:=true"
    if [$num_Args -gt 0]
    then
        args="${args} drift_type:=${1}"
    fi
    if [$num_Args -gt 1]
    then
        args="${args} enable_icp:=${2}"
    fi
    roslaunch panoptic_mapping_ros voxgraph_only.launch $args
}

function callVoxgraphSmall() {
    num_Args=$#
    args="play_rate:=5 use_noise:=true load_file:=/home/ioannis/Documents/PanopticMapping/Data/maps/flat/run1.panmap base_path:=/home/ioannis/datasets/flat_dataset/run1"
    if [$num_Args -gt 0]
    then
        args="${args} drift_type:=${1}"
    fi
    if [$num_Args -gt 1]
    then
        args="${args} enable_icp:=${2}"
    fi;
    roslaunch panoptic_mapping_ros voxgraph_only.launch $args
}

# RUN IN LARGE FLAT
SLEEPTIME=120

# callVoxgraph light &
# sleep $SLEEPTIME
# rosservice call /voxgraph_mapper/save_pose_history_to_file "file_path: '/home/ioannis/datasets/voxgraph_traj_light.bag'"
# killRos

# callVoxgraph moderate &
# sleep $SLEEPTIME
# rosservice call /voxgraph_mapper/save_pose_history_to_file "file_path: '/home/ioannis/datasets/voxgraph_traj_moderate.bag'"
# killRos


# callVoxgraph strong &
# sleep $SLEEPTIME
# rosservice call /voxgraph_mapper/save_pose_history_to_file "file_path: '/home/ioannis/datasets/voxgraph_traj_strong.bag'"
# killRos

# callVoxgraph severe &
# sleep $SLEEPTIME
# rosservice call /voxgraph_mapper/save_pose_history_to_file "file_path: '/home/ioannis/datasets/voxgraph_traj_severe.bag'"
# killRos

# RUN IN LARGE FLAT WITHOUT ICP

# callVoxgraph light "false" &
# sleep $SLEEPTIME
# rosservice call /voxgraph_mapper/save_pose_history_to_file "file_path: '/home/ioannis/datasets/voxgraph_traj_large_flat/voxgraph_traj_light.bag'"
# killRos

# callVoxgraph moderate "false" &
# sleep $SLEEPTIME
# rosservice call /voxgraph_mapper/save_pose_history_to_file "file_path: '/home/ioannis/datasets/voxgraph_traj_large_flat/voxgraph_traj_moderate.bag'"
# killRos


callVoxgraph strong "false" &
sleep $SLEEPTIME
rosservice call /voxgraph_mapper/save_pose_history_to_file "file_path: '/home/ioannis/datasets/voxgraph_traj_large_flat/voxgraph_traj_icp_off_strong.bag'"
killRos

callVoxgraph severe "false" &
sleep $SLEEPTIME
rosservice call /voxgraph_mapper/save_pose_history_to_file "file_path: '/home/ioannis/datasets/voxgraph_traj_icp_off_severe.bag'"
killRos

# RUN IN SMALL FLAT
SLEEPTIME=60
# callVoxgraphSmall strong &
# sleep $SLEEPTIME
# rosservice call /voxgraph_mapper/save_pose_history_to_file "file_path: '/home/ioannis/datasets/voxgraph_traj_small_flat/voxgraph_traj_strong.bag'"
# killRos

# callVoxgraphSmall strong &
# sleep $SLEEPTIME
# rosservice call /voxgraph_mapper/save_pose_history_to_file "file_path: '/home/ioannis/datasets/voxgraph_traj_small_flat/voxgraph_traj_severe.bag'"
# killRos

