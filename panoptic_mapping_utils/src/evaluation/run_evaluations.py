#!/bin/python3
# This script can run a batch of panoptic mapping runs and evaluations given
# a yaml file. The yaml file contains a defaults section containing all default
# arguments of the experiment and continues with the sections experimentXX
# where each experimentXX section sets the parameters of the experiment that
# it is testing. An example can be seen in
# ./panoptic_mapping_utils/config/evaluation_experiments/experiments_drift.yaml
# where we do drift experiments.
# Example run:
# ```
# cd panoptic_mapping_utils
# ./src/evaluation/run_evaluations.py /config/experiments_drift.yaml
# ```
import subprocess
import sys
import os
import time
import multiprocessing
import logging
import yaml
import rospy

logger = logging.getLogger("run_evaluations")


def getWindowId(window_name):
    """ Gets the window id in ubuntu x system given the window's name.

      Args:
          window_name (str): window's name

      Returns:
          str: window's id which is a string of a hex number.
    """
    xwininfo = subprocess.run(["xwininfo", "-name", window_name],
                              capture_output=True,
                              check=True).stdout.decode()
    logger.debug("xwininfo:")
    logger.debug(xwininfo)
    window_id = xwininfo.split('\n')[1].split(' ')[3]
    return window_id


def takeScreenshot(window_id, screenshot_path, start=None, size=None):
    """ Take a screenshot of a window and crop it given
    upper left corner (start) and area size of its
    final cropped form. The result is stored under screenshot_path.

    Args:
        screenshot_path (str): absolute path to store the screenshot
        start (list, optional): upper left corner of the cropped area in the
                                original screenshot. Defaults to [600,250].
        size (list, optional): size of the cropped area.
                            Defaults to [1740,1120].
    """
    if start is None:
        start = [600, 250]
    if size is None:
        size = [1740, 1120]
    crop_arg = "-crop " + str(size[0]) + "x" + str(size[1]) + "+" + str(
        start[0]) + "+" + str(start[1])
    os.system("import -window " + window_id + " " + crop_arg + " " +
              screenshot_path)


def closeExperiment():
    """ Close experiment by first signaling a shutdown of all ros application
     and then killing the rosmaster and roscore
    """
    rospy.signal_shutdown("experiment_process_shutdown")
    os.system("pkill -9 rosmaster")
    os.system("pkill -9 roscore")


def runEvaluation(map_path):
    """ Launched the panoptic mapping evaluation scripts
    given the path of the map to evaluate.

    Args:
      map_path (str): path of the map to evaluate.
    """
    logger.info("launching map evaluation of map_file: %s", map_path)
    os.system("roslaunch panoptic_mapping_utils evaluate_panmap.launch " +
              "map_file:=%s" % map_path)


def evaluateAfterMapIsBuilt(experiment_name,
                            map_file_path,
                            window_name,
                            screenshot_name,
                            screenshots_dir="",
                            closing_func=None):
    """ This waits for the map of the panoptic mapping to be built
    and afterwards takes a screenshot of the rviz window and runs
    an evaluation of the map.

    Args:
    experiment_name (str): name of the experiment,
                        to be used for logging purposes
    map_file_path (str): Where to store the map file
    window_name (str): name of the window to take a screenshot from
    screenshot_name (str): name of the screenshot image file.
                    If it doesn't end with .png, a .png will be appended.
    screenshots_dir (str, optional): directory to store screenshots to.
                                     Defaults to "".
    closing_func (function, optional): function to launch for closing the
             experiments. If None, closeExperiment is used. Defaults to None.
    """
    if os.path.exists(map_file_path):
        os.system("rm %s" % map_file_path)
    # wait till the map file appears
    while not os.path.exists(map_file_path):
        time.sleep(20)
    logger.info("taking screenshot for %s", experiment_name)
    if not screenshot_name.endswith(".png"):
        screenshot_name += ".png"
    screenshot_path = os.path.join(screenshots_dir, screenshot_name)
    window_id = getWindowId(window_name)
    takeScreenshot(window_id, screenshot_path)
    logger.info("launching evaluations for %s", experiment_name)
    runEvaluation(map_path=map_file_path)
    # stop all ros processes
    if closing_func is None:
        closeExperiment()
    else:
        closing_func()


def runExperiment(yaml_data, experiment_index, experiments_dir=""):
    """ This runs a single experiment provided the data in the yaml file
    and the experiment index.

    Args:
        yaml_data (dict): The configuration data for the batch
                          of experiments to be conducted.
        experiment_index (int): The index of this experiment to be launched
        experiments_dir (str, optional): If not empty this is used as the
                                directory to store experiment results to.
                                Defaults to "".
    """
    args_experiment = yaml_data['defaults'].copy()
    index = experiment_index + 1
    args_specific = yaml_data['experiment' + str(index)]
    for k, v in args_specific.items():
        args_experiment[k] = v
    base_name = args_experiment.pop('base_name')
    experiment_name = base_name + args_experiment['name']
    window_name = "devel_with_voxgraph.rviz - RViz"
    save_map_path_when_finished = os.path.join(experiments_dir,
                                               experiment_name + ".panmap")
    args_experiment[
        'save_map_path_when_finished'] = save_map_path_when_finished
    # create complete call to panoptic mapping
    line_strs = [
        "%s:=%s" % (k, str(v)) for k, v in args_experiment.items()
        if k != 'name'
    ]
    line = " ".join(line_strs)
    logger.debug(line)
    complete_call = " ".join(
        ["roslaunch panoptic_mapping_ros", yaml_data['launch_file'], line])
    logger.debug(complete_call)

    # process functions
    def start_panoptic():
        os.system(complete_call)

    def start_evaluation_wait():
        evaluateAfterMapIsBuilt(experiment_name, save_map_path_when_finished,
                                window_name,
                                experiment_name + str(time.time()),
                                experiments_dir)

    # create and start processes
    p_1 = multiprocessing.Process(name="panoptic_mapping_experiment_process",
                                  target=start_panoptic)
    p_2 = multiprocessing.Process(
        name="evaluation_after_time_experiment_process",
        target=start_evaluation_wait)
    p_1.start()
    p_2.start()
    p_2.join()
    p_1.join()
    # main_proc.send_signal(asyncio.CTRL_C_EVENT)


def main():
    """ Launches a batch of experiments that evaluate panoptic mapping
    under different settings. In case no experiments directory is provided,
    the $HOME/datasets/<first-argument> is used.

    Args:
        argv (list, optional): It contains from 2 to 3 elements.
            Element [1] is the path of the yaml config file
            and element [2] can be the experiments directory.
            Defaults to sys.argv.
    """
    argv = sys.argv
    if len(argv) < 2:
        logger.error("Not enough arguments")
        return
    if len(argv) > 2:
        experiments_dir = argv[2]
    else:
        experiments_dir = os.path.join(os.getenv('HOME'), "datasets/",
                                       argv[1].strip()[:-len(".yaml")])
        logger.info("experiments directory set to = %s", experiments_dir)
    if not os.path.exists(experiments_dir):
        os.makedirs(experiments_dir)
    yaml_data = {}
    with open(argv[1].strip(), 'rb') as fr:
        yaml_data = yaml.load(fr, Loader=yaml.FullLoader)
    num_experiments = len(
        [k for k in yaml_data.keys() if k.startswith("experiment")])
    for i in range(num_experiments):
        runExperiment(yaml_data, i, experiments_dir)


if __name__ == "__main__":
    main()
