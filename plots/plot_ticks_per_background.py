#!/usr/bin/env python
# pylint: skip-file
import os
import json
import numpy as np
from mpl_toolkits import mplot3d  # pylint: disable=W0611
from matplotlib import pyplot as plt

import get_experiment_results


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