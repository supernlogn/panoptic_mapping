#!/usr/bin/env python
#
# Example usage for generating a noisy signal
#  python drift_generator.py 0.01
#
# This will generate a noise signal of length (10000) with gaussian distribution N(0, 0.01).
# 
# 
from genericpath import isdir
import os
import sys
import numpy as np

from matplotlib import pyplot as plt
import json
from pathlib import Path
from typing import List, final
from numpy.lib.npyio import load

import rospy
from geometry_msgs.msg import PoseStamped
import scipy
from scipy.spatial.transform import Rotation as scRotation




class DriftGenerator(object):
    _x_index = 0
    _y_index = 1
    _z_index = 2

    def get_param_names() -> List[str]:
        """
        Returns:
            List[str]: All param names available by the DriftGenerator class
        """
        return ["max_noise_length",
                "pose_noise_sigma",
                "velocity_noise_sampling_frequency_hz",
                "use_as_generator",
                "save_to_file",
                "load_from_file",
                "save_history",
                "load_file_path",
                "save_file_path",
                "history_file_path"]
    
    def __init__(self, **driftParams:dict):
        """
        """
        self.max_noise_length = driftParams.get("max_noise_length", 10000)
        self.pose_noise_sigma = float(driftParams.get("pose_noise_sigma", 0.5))
        self.velocity_noise_sampling_frequency_hz = driftParams.get("velocity_noise_sampling_frequency_hz", 1)
        self.velocity_noise_sampling_period =  1/self.velocity_noise_sampling_frequency_hz
        self.use_as_generator = driftParams.get("use_as_generator", True)
        self.save_to_file = driftParams.get("save_to_file", True)
        self.load_from_file = driftParams.get("load_from_file",False)
        self.save_history = driftParams.get("save_history", True)
        self.load_file_path = Path(driftParams.get("load_file_path",""))
        self.save_file_path = Path(driftParams.get("save_file_path",""))
        self.history_file_path = Path(driftParams.get("history_file_path", ""))
        assert(not(self.load_from_file and self.save_to_file)), "Cannot load from file and save, use shells cp functionality for that"

        if self.load_from_file:
            self.load_noise_from_file()

        self.noiseIndex:int = 0
        
        if(self.use_as_generator):
            # no need to use ros, it will only generate the noise signal
            self.generate_drift_signal()
            if(self.save_to_file):
                self.save_noise_to_file()
        if self.save_history:
            self.time_history = []
            self.orig_history = []
            self.noise_history = []

    def add_drift_to_position(self, timeNow:float, position:np.ndarray, rotation:np.ndarray):
        if self.load_from_file:
            assert(type(self.total_noise) == np.ndarray), "total noise has not been created"
            newPosition = np.copy(position)
            newRotation = rotation
            newPosition[self._x_index] += self.total_noise[self.noiseIndex][0]
            newPosition[self._y_index] += self.total_noise[self.noiseIndex][1]
            newPosition[self._z_index] += self.total_noise[self.noiseIndex][2]
            # we don't edit w_index
            # pose_stamped.pose.orientation.x += self.total_noise[self.noiseIndex][3] * 1000
            # pose_stamped.pose.orientation.y += self.total_noise[self.noiseIndex][4] * 1000
            # pose_stamped.pose.orientation.z += self.total_noise[self.noiseIndex][5] * 1000
            self.noiseIndex = (self.noiseIndex + 1) % self.total_noise.shape[0]
        else:
            pass
        if self.save_history:
            self.time_history.append(timeNow)
            self.orig_history.append(position)
            self.noise_history.append(newPosition)
        
        return newPosition, newRotation

    def add_drift_to_pose(self, pose_stamped:PoseStamped):
        """ create the noisy pose and publish it as a ros message.
            2 modes offered:
            (use_velocity = true)
            pose_in + add_noise + multNoise(pose_in, past_poses) ---> noisy_pose
            note: multNoise is a function of current pose and all previous poses.
            (use_velocity = false)
            pose_in + add_noise ---> noisy_pose
        """
        if self.load_from_file:
            assert(type(self.total_noise) == np.ndarray), "total noise has not been created"
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.pose.position.x += self.total_noise[self.noiseIndex][0]
            pose_stamped.pose.position.y += self.total_noise[self.noiseIndex][1]
            pose_stamped.pose.position.z += self.total_noise[self.noiseIndex][2]
            pose_stamped.pose.orientation.x += self.total_noise[self.noiseIndex][3]
            pose_stamped.pose.orientation.y += self.total_noise[self.noiseIndex][4]
            pose_stamped.pose.orientation.z += self.total_noise[self.noiseIndex][5]
            self.noiseIndex = (self.noiseIndex + 1) % self.total_noise.shape[0]
        else:
            # adds noise in real time
            now = pose_stamped.header.stamp
            pose_msg = PoseStamped()
            pose_msg.header.stamp = now
            pose_msg.header.frame_id = self.global_frame_name
            pose_msg.pose.position.x = pose_data[3]
            pose_msg.pose.position.y = pose_data[7]
            pose_msg.pose.position.z = pose_data[11]
            pose_msg.pose.orientation.x = rotation[0]
            pose_msg.pose.orientation.y = rotation[1]
            pose_msg.pose.orientation.z = rotation[2]
            pose_msg.pose.orientation.w = rotation[3]

        return pose_stamped
    

    def generate_drift_signal(self):
        """ generate a whole drift signal that can be saved in a file and be used later """
        self.generate_pose_noise()
        self.generate_velocity_noise()
        self.total_noise = self.pose_noise

    def generate_pose_noise(self):
        """ Generate the additive noise which is irrelevant of the robot's motion """
        sigma = np.diag([self.pose_noise_sigma] * 6)
        generated_noise = np.random.randn(self.max_noise_length, 6) # white gaussian noise
        self.pose_noise = np.clip(generated_noise, -3*self.pose_noise_sigma, 3*self.pose_noise_sigma)
    
    def generate_velocity_noise(self, timestamps: np.array, ground_truth_poses: np.array):
        """ Generative multiplicative noise which is relevant of the current speed """
        # TODO(supernlogn)
        self.velocity_noise = []
        sigma = np.diag([self.pose_noise_sigma] * 3)
        timestamp_length = len(timestamps)
        integrated_pose_drift_ = np.identity(4)
        current_linear_velocity_noise_sample_W_ = np.zeros(3)
        last_velocity_noise_sampling_time_ = timestamps[0]
        # generate random vectors from white gaussian noise
        for i in range(1, timestamp_length):
            delta_t = timestamps[i] - timestamps[i-1]
            current_timestamp = timestamps[i]
            current_ground_truth_pose = ground_truth_poses[i]
            if (last_velocity_noise_sampling_time_ + self.velocity_noise_sampling_period < current_timestamp):
                #  Sample the linear velocity noise in body frame
                current_linear_velocity_noise_sample_B_ = np.random.randn(3) * self.velocity_std + self.velocity_mean
                current_linear_velocity_noise_sample_W_ = current_ground_truth_pose
                current_angular_velocity_noise_sample_W_ = np.array([0.0,0.0, np.random.randn(1)* self.velocity_std + self.velocity_mean ])
            # integrate the drift
            drift_delta_W_vec = np.concatenate([current_linear_velocity_noise_sample_W_, 
                                                current_angular_velocity_noise_sample_W_])
            drift_delta_W_vec *= delta_t
            drift_delta_W = DriftGenerator.Transformation_exp(drift_delta_W_vec)
            integrated_pose_drift_ = drift_delta_W * integrated_pose_drift_
            # Draw a random pose noise sample
            pose_noise_B_vec = np.random.randn(6)
            current_pose_noise = DriftGenerator.Transformation_exp(pose_noise_B_vec)
            # Update the current simulated pose
            self.velocity_noise.append(
                integrated_pose_drift_ * current_ground_truth_pose * current_pose_noise
            )

    def get_rotation_matrix_from_pose(self, pose_matrix:np.ndarray):
        
        return pose_matrix
    

    def Transformation_exp(v: np.ndarray):
        vlen = len(v)
        assert(vlen == 6)
        v_measure = np.linalg.norm(v)
        r = scRotation.from_rotvec(v[3:])
        v_vert = np.reshape(v[0:3], [3,1])
        M = np.vstack(np.hstack([r.as_matrix(), v_vert]), np.ones((1,4)))
        return scipy.linalg.expm(M)


    def save_history_to_file(self):
        """ Save the samples of the original signal and the noisy signal
        to a file specified in the DriftGenerator parameters.
        """
        assert(not(self.history_file_path.is_dir())), f"{self.history_file_path} needs to be a file"
        with open(str(self.history_file_path), "w") as fw:
            json.dump({
                "timestamps": list(self.time_history),
                "pose_noise_sigma": float(self.pose_noise_sigma),
                "orig_history": np.reshape(self.orig_history, [-1]).tolist(),
                "noise_history": np.reshape(self.noise_history, [-1]).tolist()
            }, fw)
        rospy.loginfo(f"path history of noisy signal and original signal has been saved to {self.history_file_path}")

    def save_noise_to_file(self):
        """ save noise generated data to a file """
        assert(not(self.save_file_path.is_dir())), f"{self.save_file_path}: please provide a file not a directory"
        with open(str(self.save_file_path), "w") as fw:
            json.dump({
                "max_noise_length": self.max_noise_length,
                "pose_noise_sigma": self.pose_noise_sigma,
                "pose_noise": np.reshape(self.pose_noise, [-1]).tolist(),
                "velocity_noise": [],
                "total_noise": self.total_noise
            }, fw)
        rospy.loginfo(f"noise signal is saved to {self.save_file_path}")

    def load_noise_from_file(self):
        """ load noisy data from a file """
        assert(self.load_file_path.exists()), f"{self.load_file_path} does not exist"
        loadedData:dict = {}
        with open(str(self.load_file_path), "r") as fr:
            loadedData:dict = json.load(fr)
        self.max_noise_length = loadedData.get("max_noise_length")
        self.pose_noise_sigma = loadedData.get("pose_noise_sigma")
        self.pose_noise = np.reshape(np.array(loadedData.get("pose_noise")), [-1, 6])
        self.velocity_noise = np.array(loadedData.get("velocity_noise", []))
        self.total_noise = self.total_noise
        rospy.loginfo(f"Noise loaded from file {self.load_file_path}")


def mainGenerator(args):
    dG = DriftGenerator(save_file_path="temp.json", pose_noise_sigma=0.1)


def mainRos(_):
    rospy.init_node('drift_generator')
    dG = DriftGenerator(load_file_path="/home/ioannis/catkin_ws/src/panoptic_mapping/panoptic_mapping_utils/src/flat_dataset/temp.json", 
                        load_from_file=True, 
                        save_to_file=False, 
                        use_as_generator=False)
    rospy.spin()

def main(args):
    argv = {arg.split("=")[0]:arg.split("=")[1] for arg in args[1:]}
    pose_noise_sigma = argv.get("pose_noise_sigma", 0.5)
    dG = DriftGenerator(
                    save_file_path=f"/home/ioannis/datasets/noise/noise_p{pose_noise_sigma}.json", 
                    load_from_file=False, 
                    save_to_file=True, 
                    use_as_generator=True,
                    pose_noise_sigma=pose_noise_sigma)


if __name__ == "__main__":
    main(sys.argv)
