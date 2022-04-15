#!/bin/env python3
# pylint: skip-file
import os, sys


def main(args):
    experiments_dir = args[1]
    exp_dirs = os.listdir(experiments_dir)
    for dir_ in exp_dirs:
        files = os.listdir(os.path.join(experiments_dir, dir_))
        print(files)
        panmap_files = [
            f for f in files
            if (f.endswith(".panmap")
                and not (f.endswith("evaluation_mean.panmap")))
        ]
        assert (panmap_files != []), "ERROR no panmap file found"
        panmap_file = os.path.join(experiments_dir, dir_, panmap_files[0])
        os.system(
            "roslaunch panoptic_mapping_utils evaluate_panmap.launch map_file:="
            + panmap_file)


if __name__ == "__main__":
    main(sys.argv)
