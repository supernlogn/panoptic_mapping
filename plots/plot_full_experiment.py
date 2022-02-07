#!/usr/bin/env python
import os
import sys
import logging
import plot_trajectories

logger = logging.getLogger()


def main():
    if len(sys.argv) < 2:
        logger.error("Need yaml path")
        exit(1)
    fullExperimentYaml = sys.argv[1]
    if not os.path.exists(fullExperimentYaml):
        logger.error("File %s does not exist", fullExperimentYaml)
        exit(2)
    plot_trajectories.plotFullExperiment(fullExperimentYaml)


if __name__ == "__main__":
    main()
