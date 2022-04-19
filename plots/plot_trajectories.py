# pylint: skip-file
from enum import unique
import json
import csv
import math
from typing import final
import numpy as np
from mpl_toolkits import mplot3d  # pylint: disable=W0611
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation
import os
import math

PI = math.pi

import rosbag
import yaml

import get_experiment_results

T_C_R = np.array([[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1]])

plt.rcParams['text.usetex'] = True


# rotations around axis
def createArrayAroundZ(angle):
    cosAngle = math.cos(angle)
    sinAngle = math.sin(angle)
    return np.array([[cosAngle, -sinAngle, 0], [sinAngle, cosAngle, 0],
                     [0, 0, 1]])


def createArrayAroundY(angle):
    cosAngle = math.cos(angle)
    sinAngle = math.sin(angle)
    return np.array([[cosAngle, 0, sinAngle], [0, 1, 0],
                     [-sinAngle, 0, cosAngle]])


def createArrayAroundX(angle):
    cosAngle = math.cos(angle)
    sinAngle = math.sin(angle)
    return np.array([[1, 0, 0], [0, cosAngle, -sinAngle],
                     [0, sinAngle, cosAngle]])


def getYawPitchRoll(q):
    # These are with false order
    yaw = math.atan2(2.0 * (q.y * q.z + q.w * q.x),
                     q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z)
    pitch = math.asin(-2.0 * (q.x * q.z - q.w * q.y))
    roll = math.atan2(2.0 * (q.x * q.y + q.w * q.z),
                      q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)
    return yaw, pitch, roll


def getGroundTruthTrajectory(file_path):
    ret1 = []
    ret2 = []
    ret3 = []
    ret1, _ = getTrajectoryTransformsFromBagFile(
        file_path, topic_name="data/pose_ground_truth")
    ret2, ret3 = getTrajectoryTransformsFromBagFile(file_path,
                                                    topic_name="data/pose")
    if ret1.size == 0:
        ret1, _ = getTrajectoryTransformsFromBagFile(
            file_path, topic_name="/data/pose_ground_truth")
        ret2, ret3 = getTrajectoryTransformsFromBagFile(
            file_path, topic_name="/data/pose")
    return np.array(ret1), np.array(ret2), np.array(ret3)


def getTrajectoryFromBagFile(file_path, topic_name='pose_history'):
    ret = []
    ret2 = []
    with rosbag.Bag(file_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            yaw, pitch, roll = getYawPitchRoll(msg.pose.orientation)
            ret.append([
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                yaw, pitch, roll
            ])
            ret2.append(t.to_sec())
    return np.array(ret), np.array(ret2)


def getTrajectoryTransformsFromBagFile(file_path, topic_name='pose_history'):
    ret = []
    ret2 = []
    with rosbag.Bag(file_path, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            yaw, pitch, roll = getYawPitchRoll(msg.transform.rotation)
            ret.append([
                msg.transform.translation.x, msg.transform.translation.y,
                msg.transform.translation.z, yaw, pitch, roll
            ])
            ret2.append(t.to_sec())
    return np.array(ret), np.array(ret2)


def plotPerAxis(gt_tr,
                wd_tr,
                op_tr,
                voxgraph_tr=np.array([]),
                N=None,
                times={}):
    if N is None and voxgraph_tr.size != 0:
        N = min(len(gt_tr[:, 1]), len(wd_tr[:, 1]), len(voxgraph_tr[:, 1]))
    elif N is None:
        N = min(len(gt_tr[:, 1]), len(wd_tr[:, 1]))
    n1_ = np.arange(0, N)
    n2_ = np.arange(0, N)
    if len(times) > 0:
        n1_ = times['panoptic_times'][:-1]
        n2_ = times['voxgraph_times'][:-1]
        N = -1
    fig, axes = plt.subplots(6, 1, figsize=(4**3, 3**3), sharex=True)

    def plotAxisWithIndex(index, label):
        axes[index].plot(n1_,
                         wd_tr[:N, index],
                         'gray',
                         label='with drift',
                         linewidth=5)
        axes[index].plot(n1_, gt_tr[:N, index], 'green', label='groundtruth')
        axes[index].plot(n1_, op_tr[:N, index], 'red', label='optimized')
        if voxgraph_tr.size > 0:
            axes[index].plot(n2_,
                             voxgraph_tr[:N, index],
                             'blue',
                             label='voxgraph')
        axes[index].set_ylabel(label)

    fig.suptitle("Trajectory per coordinate")
    plotAxisWithIndex(0, 'x[m]')
    plotAxisWithIndex(1, 'y[m]')
    plotAxisWithIndex(2, 'z[m]')
    plotAxisWithIndex(3, 'yaw[rad]')
    plotAxisWithIndex(4, 'pitch[rad]')
    plotAxisWithIndex(5, 'roll[rad]')
    axes[0].legend(loc='upper right', bbox_to_anchor=(1, 0.5))
    if len(times) > 0:
        axes[-1].set_xlabel('sec')
    else:
        axes[-1].set_xlabel('pose idx')
    fig.tight_layout()
    # plt.show()
    return fig


def plotPerAxisPlusPolarAngles(gt_tr,
                               wd_tr,
                               op_tr,
                               voxgraph_tr=np.array([]),
                               mid_poses=np.array([]),
                               N=None,
                               times={},
                               include_optimized=True):
    if N is None and voxgraph_tr.size != 0:
        print(wd_tr.shape, gt_tr.shape)
        N = min(len(gt_tr[:, 1]), len(wd_tr[:, 1]), len(voxgraph_tr[:, 1]))
    elif N is None:
        N = min(len(gt_tr[:, 1]), len(wd_tr[:, 1]))
    n1_ = np.arange(0, N)
    n2_ = np.arange(0, N)
    use_mid_poses = True
    if len(times) > 0:
        N = min(N, len(times['panoptic_times']), len(times['voxgraph_times']))
        n1_ = times['panoptic_times'][:N]
        n2_ = times['voxgraph_times'][:N]
        use_mid_poses = mid_poses.size > 0
    else:
        use_mid_poses = False
    fig1, axes1 = plt.subplots(3, 1, figsize=(4**2, 3**2), sharex=True)

    def plotAxisWithIndex(ticks, axes, index, label):
        axes[index % len(axes)].plot(ticks[:N],
                                     wd_tr[:N, index],
                                     'gray',
                                     label='with drift',
                                     linewidth=5)
        axes[index % len(axes)].plot(ticks,
                                     gt_tr[:N, index],
                                     'green',
                                     label='groundtruth')
        if include_optimized:
            axes[index % len(axes)].plot(ticks,
                                         op_tr[:N, index],
                                         'red',
                                         label='optimized')
        if voxgraph_tr.size > 0:
            axes[index % len(axes)].plot(n2_,
                                         voxgraph_tr[:N, index],
                                         'blue',
                                         label='voxgraph')
        if use_mid_poses:
            axes[index % len(axes)].scatter(times['mid_poses_times'],
                                            mid_poses[:, index],
                                            s=200.0,
                                            color='orange',
                                            marker='+',
                                            label='midposes')
        axes[index % len(axes)].set_ylabel(label)

    def plotPolarAxisWithIndex(ticks, axes, index, label):
        axes[index % len(axes)].plot(wd_tr[:N, index],
                                     ticks,
                                     'gray',
                                     label='with drift',
                                     linewidth=5)
        axes[index % len(axes)].plot(gt_tr[:N, index],
                                     ticks,
                                     'green',
                                     label='groundtruth')
        if include_optimized:
            axes[index % len(axes)].plot(op_tr[:N, index],
                                         ticks,
                                         'red',
                                         label='optimized')
        if voxgraph_tr.size > 0:
            axes[index % len(axes)].plot(voxgraph_tr[:N, index],
                                         n2_,
                                         'blue',
                                         label='voxgraph')

    plotAxisWithIndex(n1_, axes1, 0, 'x[m]')
    plotAxisWithIndex(n1_, axes1, 1, 'y[m]')
    plotAxisWithIndex(n1_, axes1, 2, 'z[m]')
    fig2, axes2 = plt.subplots(3,
                               1,
                               figsize=(4**3, 3**3),
                               sharex=True,
                               subplot_kw={'projection': 'polar'})

    plotPolarAxisWithIndex(n1_, axes2, 3, 'yaw[rad]')
    plotPolarAxisWithIndex(n1_, axes2, 4, 'pitch[rad]')
    plotPolarAxisWithIndex(n1_, axes2, 5, 'roll[rad]')

    fig1.suptitle("Trajectory per coordinate")
    fig2.suptitle("Trajectory per coordinate")
    axes1[0].legend(loc='upper right', bbox_to_anchor=(1, 0.5))
    axes2[0].legend(loc='upper right', bbox_to_anchor=(1, 0.5))
    for i in range(len(axes2)):
        axes1[i].grid(True)
        axes2[i].grid(True)
    if len(times) > 0:
        axes1[-1].set_xlabel('sec')
    else:
        axes1[-1].set_xlabel('pose idx')
    fig1.tight_layout()
    fig2.tight_layout()
    # plt.show()
    return fig1, fig2


def computeTrajectorySynchronizedLength(gt_tr,
                                        wd_tr,
                                        voxgraph_tr,
                                        times,
                                        N=None,
                                        **kwargs):
    if N is None and voxgraph_tr.size != 0:
        print(wd_tr.shape, gt_tr.shape)
        N = min(len(gt_tr[:, 1]), len(wd_tr[:, 1]), len(voxgraph_tr[:, 1]))
    elif N is None:
        N = min(len(gt_tr[:, 1]), len(wd_tr[:, 1]))
    n1_ = np.arange(0, N)
    n2_ = np.arange(0, N)
    pair_time_matches = []
    if len(times) > 0:
        N = min(N, len(times['panoptic_times']), len(times['voxgraph_times']))
        n1_ = times['panoptic_times'][:N]
        n2_ = times['voxgraph_times'][:N]

        t1_index = 0
        t2_index = 0
        # construct pairs of times of the two timeseries
        while t1_index != N and t2_index != N:
            if n1_[t1_index] < n2_[t2_index]:
                t1_index += 1
            elif n1_[t1_index] > n2_[t2_index]:
                t2_index += 1
            else:
                pair_time_matches.append([t1_index, t2_index])
                t1_index += 1
                t2_index += 1
        pair_time_matches = np.array(pair_time_matches, dtype=np.int)
        N = len(pair_time_matches)
        n1_ = n1_[pair_time_matches[:, 0]]
        n2_ = n2_[pair_time_matches[:, 1]]
    return N, n1_, n2_, pair_time_matches


def plotXYZErrorsPerAxis(gt_tr,
                         wd_tr,
                         op_tr,
                         voxgraph_tr=np.array([]),
                         N=None,
                         times={},
                         include_optimized=True,
                         **kwargs):
    N, n1_, n2_, pair_time_matches = computeTrajectorySynchronizedLength(
        gt_tr, wd_tr, voxgraph_tr, times, N)
    if len(times) > 0:
        wd_tr = wd_tr[:N, :] - gt_tr[:N, :]
        if include_optimized:
            op_tr = op_tr[:N, :] - gt_tr[:N, :]
        if voxgraph_tr.size > 0:
            voxgraph_tr = voxgraph_tr[pair_time_matches[:, 1], :] - gt_tr[
                pair_time_matches[:, 0], :]
    else:
        wd_tr = wd_tr[:N, :] - gt_tr[:N, :]
        if include_optimized:
            op_tr = op_tr[:N, :] - gt_tr[:N, :]
        if voxgraph_tr.size > 0:
            voxgraph_tr = voxgraph_tr[:N, :] - gt_tr[:N, :]
    fig, axes = plt.subplots(3, 1, figsize=(4**2, 3**2), sharex=True)

    def plotAxisWithIndex(ticks, axes, index, label):
        axes[index % len(axes)].plot(ticks[:N],
                                     wd_tr[:N, index],
                                     'gray',
                                     label='with drift',
                                     linewidth=5)
        if include_optimized:
            axes[index % len(axes)].plot(ticks[:N],
                                         op_tr[:N, index],
                                         'red',
                                         label='optimized')
        if voxgraph_tr.size > 0:
            axes[index % len(axes)].plot(n2_,
                                         voxgraph_tr[:N, index],
                                         'blue',
                                         label='voxgraph')
        axes[index % len(axes)].set_ylabel(label)
        # for item in ([axes[index % len(axes)].title, axes[index % len(axes)].xaxis.label, axes[index % len(axes)].yaxis.label] +
        #             axes[index % len(axes)].get_xticklabels() + axes[index % len(axes)].get_yticklabels()):
        #     item.set_fontsize(100)

    plotAxisWithIndex(n1_, axes, 0, r'$\Delta x$[m]')
    plotAxisWithIndex(n1_, axes, 1, r'$\Delta y$[m]')
    plotAxisWithIndex(n1_, axes, 2, r'$\Delta z$[m]')
    fig.suptitle("Trajectory Errors per coordinate")
    axes[0].legend(loc='upper right', bbox_to_anchor=(1, 0.5))
    for i in range(len(axes)):
        axes[i].grid(True)
        if len(times) > 0:
            axes[-1].set_xlabel('sec')
        else:
            axes[-1].set_xlabel('pose idx')
    fig.tight_layout()
    return fig


def getErrorFromManyExperiments(all_trajectories):
    total_errors = {}
    for name, info in all_trajectories.items():
        r = computeTrajectorySynchronizedLength(**info)
        N = r[0]
        diff = info['op_tr'][:N, :] - info['gt_tr'][:N, :]
        residual = diff * diff
        total_error = np.sqrt(np.sum(residual) / N)
        total_errors[name] = total_error
    return total_errors


def trajectoryVSplot(r1, r2, title="Trajectory Errors Comparison"):
    assert r1.keys() == r2.keys(), "different number of keys"

    fig = plt.figure(figsize=(16, 9))
    ax = plt.axes()
    width = 0.8  # the width of the bars
    keys = [k for k in r1.keys() if not 'num_points' in k]
    vals = np.array([float(r1[k]) for k in keys])
    vals2 = np.array([float(r2[k]) for k in keys])
    x = np.arange(0, len(keys))
    ax.bar(x=(x - width / 2), height=vals, width=width / 2, label='wo interp')
    ax.bar(x=(x), height=vals2, width=width / 2, label='w interp')
    for i in ax.patches:
        plt.text(i.get_x() + width / 8,
                 i.get_height(),
                 str(round(i.get_height(), 3)),
                 fontsize=10,
                 fontweight='bold',
                 color='grey')
    ax.set_title(title)
    ax.set_xticks(x)
    ax.set_xticklabels(keys)
    ax.legend()
    fig.tight_layout()
    plt.show()


def plotErrorForManyExperiments(all_trajectories,
                                title='Mapping Evaluations',
                                error_type='RMSE[m]'):
    fig = plt.figure(2)
    total_errors = getErrorFromManyExperiments(all_trajectories)
    x = np.arange(4)
    width = 0.8
    drift_labels = ['light', 'moderate', 'strong', 'severe']
    labels = set([k[k.index('_') + 1:] for k in all_trajectories.keys()])
    fig, ax = plt.subplots()
    num_labels = len(labels)
    for i, label in enumerate(labels):
        vals = []
        for j, d_label in enumerate(drift_labels):
            vals.append(total_errors[d_label + '_' + label])
        ax.bar(x=(x - width / 2 + i * width / num_labels),
               height=vals,
               width=width / num_labels,
               label=label)
    for i in ax.patches:
        plt.text(i.get_x() + width / 8,
                 i.get_height(),
                 str(round(i.get_height(), 3)),
                 fontsize=10,
                 fontweight='bold',
                 color='grey')
    ax.set_title(title)
    ax.set_ylabel(error_type)
    ax.set_xticks(x)
    ax.set_xticklabels(drift_labels)
    ax.legend()
    fig.tight_layout()
    return fig


def getExperimentResults(yaml_file_path):
    dir_final_name = os.path.basename(yaml_file_path)[:-len(".yaml")]
    with open(yaml_file_path, 'r') as fr:
        yaml_data = yaml.load(fr, yaml.FullLoader)
    base_experiments_dir = os.path.join(os.getenv("HOME"), 'datasets',
                                        dir_final_name)
    experiments_results = get_experiment_results.getExperimentResults(
        base_experiments_dir)
    return experiments_results


def plotMappingErrorForManyExperiments(yaml_file_path,
                                       labels,
                                       title='Mapping Evaluations',
                                       error_type='RMSE[m]',
                                       use_label_numbers=True):
    fig = plt.figure()
    experiments_results = getExperimentResults(yaml_file_path)
    evaluations = [
        float(er['panmap'][error_type]) for er in experiments_results
    ]
    x = np.arange(4) * 4
    width = 0.8 * 4
    drift_labels = ['light', 'moderate', 'strong', 'severe']
    # labels = set([k[k.index('_')+1:] for k in all_trajectories.keys()])
    fig, ax = plt.subplots()
    num_labels = len(labels)
    for i, label in enumerate(labels):
        vals = []
        for j, d_label in enumerate(drift_labels):
            vals.append(evaluations[i * 4 +
                                    j])  # total_errors[d_label + '_' + label])
        ax.bar(x=(x - width / 2 + i * width / num_labels),
               height=vals,
               width=width / num_labels,
               label=label)
    if use_label_numbers:
        for i in ax.patches:
            plt.text(i.get_x() + width / 8,
                     i.get_height(),
                     str(round(i.get_height(), 3)),
                     fontsize=10,
                     fontweight='bold',
                     color='grey')
    ax.set_title(title)
    ax.set_ylabel(error_type)
    ax.set_xticks(x)
    ax.set_xticklabels(drift_labels)
    ax.legend(loc='lower center')
    ax.grid()
    fig.tight_layout()
    return fig


def plotTrajectoryErrorForManyExperiments(yaml_file_path,
                                          labels,
                                          title='Trajectory Evaluations',
                                          error_type='og_rmse_pos[m]',
                                          ylabel='RMSE [m]',
                                          use_label_numbers=True):
    fig = plt.figure()
    experiments_results = getExperimentResults(yaml_file_path)
    evaluations = [
        float(er['trajectory'][error_type]) for er in experiments_results
    ]
    x = np.arange(4) * 4
    width = 0.8 * 4
    drift_labels = ['light', 'moderate', 'strong', 'severe']
    # labels = set([k[k.index('_')+1:] for k in all_trajectories.keys()])
    fig, ax = plt.subplots()
    num_labels = len(labels)
    for i, label in enumerate(labels):
        vals = []
        for j, d_label in enumerate(drift_labels):
            vals.append(evaluations[i * 4 +
                                    j])  # total_errors[d_label + '_' + label])
        ax.bar(x=(x - width / 2 + i * width / num_labels),
               height=vals,
               width=width / num_labels,
               label=label)
    if use_label_numbers:
        for i in ax.patches:
            plt.text(i.get_x() + width / 8,
                     i.get_height(),
                     str(round(i.get_height(), 3)),
                     fontsize=10,
                     fontweight='bold',
                     color='grey')
    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.set_xticks(x)
    ax.set_xticklabels(drift_labels)
    ax.legend(loc='top center')
    ax.grid()
    fig.tight_layout()
    return fig


def plotMultiFile(voxgraph_trs=[], times=[], N=None):
    if N is None != 0:
        N = np.min([len(tr[:, 0]) for tr in voxgraph_trs])
    n1_ = np.arange(0, N)
    colors = np.random.random((len(voxgraph_trs), 3))
    if len(times) > 0:
        n1_ = times[:-1]
        N = -1
    fig = plt.figure(figsize=(4**2, 3**2))
    axes = plt.axes()
    for i, tr in enumerate(voxgraph_trs):
        axes.plot(n1_, tr[:, 2], c=colors[i, :], label='tr%d' % i)
    if len(times) > 0:
        axes.set_xlabel('sec')
    else:
        axes.set_xlabel('pose idx')
    axes.set_ylabel('z[m]')
    axes.set_title('Voxblox z estimation per time')
    fig.tight_layout()
    return fig


def plot3D(gt_tr, wd_tr, op_tr, voxgraph_tr=np.array([]), N=None):
    if N is None and voxgraph_tr.size == 0:
        N = min(len(gt_tr[:, 1]), len(voxgraph_tr[:, 1]))
    elif N is None:
        N = len(gt_tr[:, 1])
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot3D(wd_tr[:N, 0],
              wd_tr[:N, 1],
              wd_tr[:N, 2],
              'gray',
              label='with drift')
    ax.plot3D(gt_tr[:N, 0],
              gt_tr[:N, 1],
              gt_tr[:N, 2],
              'green',
              label='groundtruth')
    ax.plot3D(op_tr[:N, 0],
              op_tr[:N, 1],
              op_tr[:N, 2],
              'red',
              label='optimized')
    if voxgraph_tr.size > 0:
        ax.plot3D(voxgraph_tr[:N, 0],
                  voxgraph_tr[:N, 1],
                  voxgraph_tr[:N, 2],
                  'blue',
                  label='voxgraph')
    ax.set_title('Trajectory View')
    ax.set_xlabel('x[m]')
    ax.set_ylabel('y[m]')
    ax.set_zlabel('z[m]')
    ax.legend()
    fig.tight_layout()
    plt.show()


def getTrajectories(files):
    print(files)
    gt_tr, wd_tr, gt_times = getGroundTruthTrajectory(files['ground_truth_tr'])
    op_tr, op_tr_times = getTrajectoryFromBagFile(files['optimized_tr'])
    voxgraph_tr, voxgraph_times = np.array([]), np.array([])
    if 'voxgraph_tr' in files.keys():
        voxgraph_tr, voxgraph_times = getTrajectoryFromBagFile(
            files['voxgraph_tr'])
    mid_poses, mid_poses_times = np.array([]), np.array([])
    if 'mid_poses' in files.keys():
        mid_poses, mid_poses_times = getTrajectoryFromBagFile(
            files['mid_poses'])
    return gt_tr, wd_tr, gt_times, op_tr, op_tr_times, voxgraph_tr, voxgraph_times, mid_poses, mid_poses_times


def getMultiFileTrajectory(base_path):
    """ Get trajectories from files with indexes in a directory.
    If base_path is $HOME/traj.bag
    then all $HOME/traj.bag1, $HOME/traj.bag2, ...
    will be used and their trajectories will be stored in the result.
    The returning trajectories will be in the order of the file indices.
    So:
        trajectories[0] will be the trajectory of $HOME/traj.bag0
        trajectories[1] will be the trajectory of $HOME/traj.bag1
        ...
    Args:
        base_path (str): The base path to add index tom and get trajectories

    Returns:
        list: The trajectories from the files
    """
    print(base_path)
    dir_path = os.path.dirname(base_path)
    all_dir_files = os.listdir(dir_path)
    base_path_basename = os.path.basename(base_path)
    trajectory_files = [
        os.path.join(dir_path, f) for f in all_dir_files
        if f.startswith(base_path_basename)
    ]
    trajectory_files.sort(key=lambda x: int(x[x.rfind('.bag') + len('.bag'):]))
    trajectories = []
    for file in trajectory_files:
        trajectories.append(getTrajectoryFromBagFile(file))
    return trajectories


def plot_dev():
    home_dir = os.getenv("HOME")
    gt_tr, wd_tr, _ = getGroundTruthTrajectory(home_dir + "/datasets/"
                                               "generated_path.txt")
    op_tr, op_tr_times = getTrajectoryFromBagFile(home_dir + "/datasets/"
                                                  "trajectory.in")
    voxgraph_tr, voxgraph_times = getTrajectoryFromBagFile(
        home_dir + "/datasets/voxgraph_traj.bag")
    # voxgraph_tr, voxgraph_times = getTrajectoryFromBagFile(
    #  "/home/ioannis/datasets/voxgraph_traj_small_flat/voxgraph_traj_severe.bag")
    # plot3D(gt_tr, wd_tr, op_tr)
    print(voxgraph_tr.shape)
    N = None
    d = len(op_tr) - len(voxgraph_tr)
    # gt_tr, wd_tr, op_tr = gt_tr[d:,:], wd_tr[d:,:], op_tr[d:,:]

    plotPerAxis(gt_tr,
                wd_tr,
                op_tr,
                voxgraph_tr,
                N,
                times={
                    'panoptic_times': op_tr_times,
                    'voxgraph_times': voxgraph_times
                })
    # plot3D(gt_tr, wd_tr, op_tr, voxgraph_tr)


def getTrajectoryFiles(base_dir, voxgraph_file=""):
    dirFiles = os.listdir(base_dir)
    files = {
        'ground_truth_tr': os.path.join(base_dir, "generated_path.txt"),
        'optimized_tr': os.path.join(base_dir, "trajectory.in")
    }
    if voxgraph_file == "":
        if 'voxgraph_traj.bag' in dirFiles:
            files['voxgraph_tr'] = os.path.join(base_dir, 'voxgraph_traj.bag')
    return files


def pickLogPoseGivenTransform(p, Tr):
    finalp = []
    position = np.array(Tr, np.float).dot(np.array(p[:3]))
    # just append angles
    r1 = Rotation.from_matrix(Tr)
    Tr_euler = r1.as_euler('xyz')
    angles = np.array(p[3:]) + Tr_euler
    finalp = np.hstack((position, angles))
    return finalp


def pickLogPoseGivenTransformPost(p, Tr):
    # only angles change
    r1 = Rotation.from_matrix(Tr)
    Tr_euler = r1.as_euler('xyz')
    angles = np.array(p[3:]) + Tr_euler
    position = p[:3]
    return np.hstack((position, angles))


def plotFiles(files,
              output_dir,
              figName,
              T_C_R=None,
              post_mult=False,
              include_optimized=True):
    gt_tr, wd_tr, _, op_tr, op_tr_times, voxgraph_tr, voxgraph_times, mid_poses, mid_poses_times = getTrajectories(
        files)
    if not (T_C_R is None):
        pick = T_C_R[0:3, 0:3]
        print(pick)
        if post_mult:
            voxgraph_tr2 = np.array([
                pickLogPoseGivenTransformPost(p, np.linalg.inv(pick))
                for p in voxgraph_tr
            ])
            # voxgraph_tr2 = voxgraph_tr
        else:
            voxgraph_tr2 = np.array([
                pickLogPoseGivenTransform(p, np.linalg.inv(pick))
                for p in voxgraph_tr
            ])
            # voxgraph_tr2 = voxgraph_tr
    else:
        voxgraph_tr2 = voxgraph_tr
    N = None
    print(voxgraph_tr.shape)
    fig1, fig2 = plotPerAxisPlusPolarAngles(
        gt_tr,
        wd_tr,
        op_tr,
        voxgraph_tr2,
        mid_poses,
        N,
        times={
            'panoptic_times': op_tr_times,
            'voxgraph_times': voxgraph_times,
            'mid_poses_times': mid_poses_times
        },
        include_optimized=include_optimized)
    fig3 = plotXYZErrorsPerAxis(gt_tr,
                                wd_tr,
                                op_tr,
                                voxgraph_tr2,
                                N,
                                times={
                                    'panoptic_times': op_tr_times,
                                    'voxgraph_times': voxgraph_times
                                },
                                include_optimized=include_optimized)

    fig1.savefig(os.path.join(output_dir, figName + "_xyz.png"))
    fig2.savefig(os.path.join(output_dir, figName + "_angles.png"))
    fig3.savefig(os.path.join(output_dir, figName + "_errors.png"))


def plotPerAxisDirectory(base_dir,
                         figName,
                         T_C_R=None,
                         post_mult=False,
                         include_optimized=True,
                         mid_poses_file=None):
    files = getTrajectoryFiles(base_dir)
    files['mid_poses'] = mid_poses_file
    plotFiles(files,
              base_dir,
              figName,
              T_C_R=None,
              post_mult=False,
              include_optimized=True)


def getAllExperimentTrajectories(yaml_file_path, post_mult=True):
    dir_final_name = os.path.basename(yaml_file_path)[:-len(".yaml")]
    yaml_data = {}
    with open(yaml_file_path, 'r') as fr:
        yaml_data = yaml.load(fr, yaml.FullLoader)
    base_experiments_dir = os.path.join(os.getenv("HOME"), 'datasets',
                                        dir_final_name)
    files_in_dir = os.listdir(base_experiments_dir)
    all_trajectories = {}
    for i, fd in enumerate(files_in_dir):
        experiment_path = os.path.join(base_experiments_dir, fd)
        print(experiment_path)
        if os.path.isdir(experiment_path):
            int_fd = int(fd[len('experiment'):])
            experiment_data = yaml_data['experiment' + str(int_fd + 1)]
            files = getTrajectoryFiles(experiment_path)
            gt_tr, wd_tr, _, op_tr, op_tr_times, voxgraph_tr, voxgraph_times, mid_poses, mid_poses_times = getTrajectories(
                files)
            if post_mult:
                voxgraph_tr2 = np.array([
                    pickLogPoseGivenTransformPost(
                        p, np.linalg.inv(T_C_R[0:3, 0:3])) for p in voxgraph_tr
                ])
                # voxgraph_tr2 = voxgraph_tr
            else:
                voxgraph_tr2 = np.array([
                    pickLogPoseGivenTransform(p, np.linalg.inv(T_C_R[0:3,
                                                                     0:3]))
                    for p in voxgraph_tr
                ])
            voxgraph_tr = voxgraph_tr2
            times = {
                'panoptic_times': op_tr_times,
                'voxgraph_times': voxgraph_times,
                'mid_poses_times': mid_poses_times
            }
            all_trajectories[experiment_data["name"]] = {
                "gt_tr": gt_tr,
                "wd_tr": wd_tr,
                "op_tr": op_tr,
                "times": times,
                "voxgraph_tr": voxgraph_tr,
                "mid_poses": mid_poses
            }
    return all_trajectories


def getPanmapFile(experiment_path):
    files = os.listdir(experiment_path)
    panmap_files = [
        f for f in files if (f.endswith(".panmap")
                             and not (f.endswith("evaluation_mean.panmap")))
    ]
    panmap_file = os.path.join(experiment_path, panmap_files[0])
    return panmap_file


def getExperimentCSVFileContents(experiment_path):
    files = os.listdir(experiment_path)
    csv_files = [f for f in files if f.endswith(".csv")]
    csv_file = csv_files[0]
    res = []
    with open(csv_file, 'r') as fr:
        csvreader = csv.DictReader(fr)
        for row in csvreader:
            res.append(row)
    return res


def getAllExperimentMappingErrors():
    yaml_file_path, post_mult = True
    dir_final_name = os.path.basename(yaml_file_path)[:-len(".yaml")]
    yaml_data = {}
    with open(yaml_file_path, 'r') as fr:
        yaml_data = yaml.load(fr, yaml.FullLoader)
    base_experiments_dir = os.path.join(os.getenv("HOME"), 'datasets',
                                        dir_final_name)
    files_in_dir = os.listdir(base_experiments_dir)
    all_error_infos = {}
    for i, fd in enumerate(files_in_dir):
        experiment_path = os.path.join(base_experiments_dir, fd)
        print(experiment_path)
        if os.path.isdir(experiment_path):
            int_fd = int(fd[len('experiment'):])
            experiment_data = yaml_data['experiment' + str(int_fd + 1)]
            info = getExperimentCSVFileContents(experiment_path)
            all_error_infos[experiment_data["name"]] = info
    return all_error_infos


def plotFullExperiment(yaml_file_path):
    T_C_R = np.array([[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0,
                                                                   1]])
    dir_final_name = os.path.basename(yaml_file_path)[:-len(".yaml")]
    yaml_data = {}
    with open(yaml_file_path, 'r') as fr:
        yaml_data = yaml.load(fr, yaml.FullLoader)
    base_experiments_dir = os.path.join(os.getenv("HOME"), 'datasets',
                                        dir_final_name)
    files_in_dir = os.listdir(base_experiments_dir)
    for i, fd in enumerate(files_in_dir):
        experiment_path = os.path.join(base_experiments_dir, fd)
        print(experiment_path)
        if os.path.isdir(experiment_path):
            int_fd = int(fd[len('experiment'):])
            experiment_data = yaml_data['experiment' + str(int_fd + 1)]
            figName = experiment_data['name'] + '.png'
            plotPerAxisDirectory(experiment_path,
                                 figName,
                                 T_C_R,
                                 post_mult=True)


# def main():
#     home_dir = os.getenv("HOME")
#     gt_tr, wd_tr, _ = getGroundTruthTrajectory(
#         home_dir + "/datasets/experiments_w_wo_alignment/experiment2/"
#         "generated_path.txt")
#     op_tr, op_tr_times = getOptimizedTrajectory(
#         home_dir + "/datasets/experiments_w_wo_alignment/experiment2/"
#         "trajectory.in")
#     voxgraph_tr, voxgraph_times = getVoxgraphTrajectory(
#         home_dir +
#         "/datasets/voxgraph_traj_large_flat/voxgraph_traj_icp_off_strong.bag")
#     plotPerAxis(gt_tr, wd_tr, op_tr, voxgraph_tr, N=None)
#     plt.show()
#     # plot3D(gt_tr, wd_tr, op_tr, voxgraph_tr)
#     # plot_dev()

# if __name__ == "__main__":
#     main()
