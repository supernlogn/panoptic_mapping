#!/usr/bin/env python
import csv
import os
import logging
import coloredlogs

import numpy as np
from matplotlib import pyplot as plt


def getLogger():
    ret = logging.getLogger("run_evaluations")
    ret.setLevel(logging.DEBUG)
    fmt = ('[%(asctime)s] - %(name)s -'
           ' {line:%(lineno)d} %(levelname)s - %(message)s')
    coloredlogs.install(fmt=fmt, level='DEBUG', logger=ret)
    return ret


logger = getLogger()


def getExperimentResults(base_dir):
    dir_list = os.listdir(base_dir)
    dir_list = [d for d in dir_list if d.startswith('experiment')]
    l = len(dir_list)
    experiment_dirs = ['experiment' + str(i) for i in range(l)]
    logger.info(experiment_dirs)
    experiments_results = []
    for dir_ in experiment_dirs:
        if not os.path.isdir(os.path.join(base_dir, dir_)):
            continue

        res = getDirectExperimentResults(os.path.join(base_dir, dir_))
        experiments_results.append(res)
        logger.info(experiments_results[-1])
    return experiments_results


def getDirectExperimentResults(dir_):
    trajectory_results = []
    map_evaluation_results = []
    experiment_files = os.listdir(dir_)
    trajectory_evaluation_files = [
        f for f in experiment_files if f.endswith('trajectory.out')
    ]
    if trajectory_evaluation_files:
        trajectory_evaluation_file = os.path.join(
            dir_, trajectory_evaluation_files[0])
        with open(trajectory_evaluation_file, 'r') as fr:
            reader = csv.DictReader(fr, delimiter=',')
            logger.info('opened %s', trajectory_evaluation_file)
            for line in reader:
                trajectory_results = line

    map_evaluation_files = [
        f for f in experiment_files if f.endswith('evaluation_data.csv')
    ]
    if map_evaluation_files:
        map_evaluation_file = os.path.join(dir_, map_evaluation_files[0])
        with open(map_evaluation_file, 'r') as fr:
            reader = csv.DictReader(fr, delimiter=',')
            logger.info('opened %s', map_evaluation_file)
            for line in reader:
                map_evaluation_results = line
    return {'trajectory': trajectory_results, 'panmap': map_evaluation_results}


def autolabel(rects, ax):
    """Attach a text label above each bar in *rects*, displaying its height."""
    for rect in rects:
        height = rect.get_height()
        ax.annotate(
            '{}'.format(height),
            xy=(rect.get_x() + rect.get_width() / 2, height),
            xytext=(0, 3),  # 3 points vertical offset
            textcoords="offset points",
            ha='center',
            va='bottom')


def grouped_bar_chart(arr):
    labels = ['none', 'linear', 'linearT', 'exp']
    drift_labels = ['light', 'moderate', 'strong', 'severe']
    result_per_drift = {
        'light': [arr[i] for i in range(0, len(arr), 4)],
        'moderate': [arr[i] for i in range(1, len(arr), 4)],
        'strong': [arr[i] for i in range(2, len(arr), 4)],
        'severe': [arr[i] for i in range(3, len(arr), 4)]
    }

    x = np.arange(len(labels))  # the label locations
    width = 0.35  # the width of the bars

    fig, ax = plt.subplots()
    group_rects = []

    for i, label in enumerate(drift_labels):
        group_rects.append(
            ax.bar(x - width / 2 + i * width / 2,
                   result_per_drift[label],
                   width / 2,
                   label=label))
    # Add some text for labels, title and custom x-axis tick labels, etc.
    ax.set_ylabel('RMSE[m]')
    ax.set_title('Mapping Evaluations')
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.legend()
    # for rects in group_rects:
    #     autolabel(rects, ax)
    fig.tight_layout()
    plt.show()


def grouped_line_plot(arr, title):
    labels = ['none', 'linear', 'linearT', 'exp']
    result_per_drift = {
        'light': [arr[i] for i in range(0, len(arr), 4)],
        'moderate': [arr[i] for i in range(1, len(arr), 4)],
        'strong': [arr[i] for i in range(2, len(arr), 4)],
        'severe': [arr[i] for i in range(3, len(arr), 4)]
    }
    x = np.arange(0, len(arr) // 4, 1)
    fig, ax = plt.subplots()
    for k, v in result_per_drift.items():
        ax.plot(x, v, label=k, linewidth=4, marker='D')
    ax.set_ylabel('RMSE[m]')
    ax.set_title(title)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.legend()
    fig.tight_layout()
    plt.show()


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


def get_og(d):
    return {k: v for k, v in d.items() if k.startswith('og')}


def compareTrajectories(dir1, dir2):
    assert os.path.exists(dir1), "%s does not exist" % dir1
    assert os.path.exists(dir2), "%s does not exist" % dir2
    exp_results1 = getDirectExperimentResults(dir1)
    exp_results2 = getDirectExperimentResults(dir2)
    logger.info(exp_results1)
    logger.info(exp_results2)

    trajectoryVSplot(get_og(exp_results1['trajectory']),
                     get_og(exp_results2['trajectory']))


def main():
    # experiments_results = getExperimentResults(
    #   '/home/ioannis/datasets/experiments_interpolation01/')
    # og_rmses = []
    # rec_rmses = []
    # for e_r in experiments_results:
    #     og_rmse = e_r['trajectory']['og_rmse_pos[m]']
    #     rec_rmse = e_r['panmap']['MeanError [m]']
    #     og_rmses.append(og_rmse)
    #     rec_rmses.append(rec_rmse)
    # logger.info(rec_rmses)
    # with plt.style.context('seaborn'):
    #     grouped_line_plot(og_rmses, 'Trajectory Evaluations')

    # compareTrajectories(
    #     "/home/ioannis/datasets/experiments_w_wo_interpolation/experiment2",
    #     "/home/ioannis/datasets/experiments_w_wo_interpolation/experiment7"
    # )
    pass


if __name__ == "__main__":
    main()
