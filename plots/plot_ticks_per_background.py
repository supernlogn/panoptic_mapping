#!/usr/bin/env python
# pylint: skip-file
import os
import json
import yaml
import numpy as np
from mpl_toolkits import mplot3d  # pylint: disable=W0611
from matplotlib import pyplot as plt

import get_experiment_results
import plot_trajectories


def plot_all_metrics_in_a_single_image(yaml_file_path, x_variable_name,
                                       x_variable_sorted):
    all_trajectories = plot_trajectories.getAllExperimentTrajectories(
        yaml_file_path=yaml_file_path, post_mult=True)
    dir_final_name = os.path.basename(yaml_file_path)[:-len(".yaml")]
    with open(yaml_file_path, 'r') as fr:
        yaml_data = yaml.load(fr, yaml.FullLoader)
    base_experiments_dir = os.path.join(os.getenv("HOME"), 'datasets',
                                        dir_final_name)
    experiments_results = get_experiment_results.getExperimentResults(
        base_experiments_dir)
    keys = all_trajectories.keys()
    fig, axs = plt.subplots(1, 2, figsize=(16, 9))
    error_metric = 'RMSE [m]'
    evaluations = [
        float(er['panmap'][error_metric]) for er in experiments_results
    ]
    # re arrange evaluations to be 1-1 with x_variable_sorted
    idx = []
    for val in keys:
        idx.append(x_variable_sorted.index(val))
    evaluations_ordered = [evaluations[i] for i in idx]
    # Mapping error
    axs[0].plot(x_variable_sorted,
                evaluations_ordered,
                linewidth=4,
                marker='D')
    axs[0].set_ylabel(error_metric)
    axs[0].set_xlabel(x_variable_name)
    axs[0].title.set_text('Mapping Error')

    axs[1].plot(x_variable_sorted,
                evaluations_ordered,
                linewidth=4,
                marker='D')
    axs[1].set_ylabel(error_metric)
    axs[1].set_xlabel(x_variable_name)
    axs[1].title.set_text('Trajectory Error')
    plt.subtitle("Parameter study of {}".format(x_variable_name))
    fig.tight_layout()
    return fig


# plot the metrics for the background_per_n_ticks experiment
# In this experiment we test how many frames(n_ticks)
# should a background submap include
def main():
    experiments_results = get_experiment_results.getExperimentResults(
        '/home/ioannis/datasets/background_per_n_ticks/')
    n = [10, 12, 14, 16, 18, 20, 22]

    fig, axs = plt.subplots(1, 3, figsize=(16, 9))
    error_metrics = ['RMSE [m]', 'MeanError [m]', 'StdError [m]']
    for i, error_metric in enumerate(error_metrics):
        evaluations = [
            float(er['panmap'][error_metric]) for er in experiments_results
        ]
        print(evaluations)
        axs[i].plot(n, evaluations, linewidth=4, marker='D')
        axs[i].set_ylabel(error_metric)
        axs[i].set_xlabel("change background per ticks")
    plt.suptitle('Mapping Evaluations')
    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()