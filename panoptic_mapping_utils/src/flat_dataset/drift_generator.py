import os
import sys
import numpy as np
import json
from pathlib import Path

import rospy
from geometry_msgs.msg import PoseStamped
# from sensor_msgs.msg import



class DriftGenerator(object):
    def __init__(self, **driftParams:dict):
        print("!!!!!!!!!!!!!!!STARTING THE DRIFT GENERATOR!!!!!!!!!!!!!!!")
        #     velocity_noise_sampling_period_ = 1.f / config.velocity_noise_frequency_hz
        self.max_noise_length = driftParams.get("max_noise_length",100)
        self.pose_noise_amplitude = driftParams.get("pose_noise_amplitude",0.5)
        self.useDrift = driftParams.get("useDrift",True)
        self.velocity_noise_frequency_hz = driftParams.get("velocity_noise_frequency_hz", 1)
        self.velocity_noise_sampling_period = driftParams.get("velocity_noise_sampling_period", 1/self.velocity_noise_frequency_hz)
        self.use_as_generator = driftParams.get("use_as_generator", True)
        self.save_to_file = driftParams.get("save_to_file", True)
        self.load_from_file = driftParams.get("load_from_file",False)
        self.load_file_path = Path(driftParams.get("load_file_path",""))
        self.save_file_path = Path(driftParams.get("save_file_path",""))
        assert(not(self.load_from_file and self.save_to_file)), "Cannot load from file and save, use shells cp functionality for that"

        if self.load_from_file:
            self.loadNoiseFromFile(self.load_file_path)

        self.noiseIndex = 0
        
        if(self.use_as_generator):
            self.generatePoseNoise()
            if(self.save_to_file):
                self.saveNoiseToFile(self.save_file_path)
        else:
            # no need to use ros, it will only generate the noise signal
            if(self.useDrift):
                self.pose_sub = rospy.Subscriber("~pose_clean_in", PoseStamped, self.addDriftToPose)
            else:
                self.pose_sub = rospy.Subscriber("~pose_clean_in", PoseStamped, self.poseIdentityFunction)
            self.noisy_pose_pub = rospy.Publisher("~pose", PoseStamped, queue_size=100)


    def poseIdentityFunction(self, pose_stamped:PoseStamped):
        """ This function does nothing. It is pose in ---> pose out """
        self.noisy_pose_pub.publish(pose_stamped)

    def addDriftToPose(self, pose_stamped):
        """ create the noisy pose and publish it as a ros message.
            2 modes offered:
            (use_velocity = true)
            pose_in + add_noise + multNoise(pose_in, past_poses) ---> noisy_pose
            note: multNoise is a function of current pose and all previous poses.
            (use_velocity = false)
            pose_in + add_noise ---> noisy_pose
        """
        if self.load_from_file:
            assert(self.total_noise != None)
            pose_stamped.pose.position.x += self.total_noise[self.noiseIndex][0]
            pose_stamped.pose.position.y += self.total_noise[self.noiseIndex][1]
            pose_stamped.pose.position.z += self.total_noise[self.noiseIndex][2]
            pose_stamped.pose.orientation.x += self.total_noise[self.noiseIndex][3]
            pose_stamped.pose.orientation.y += self.total_noise[self.noiseIndex][4]
            pose_stamped.pose.orientation.z += self.total_noise[self.noiseIndex][5]
            pose_stamped.pose.orientation.w += self.total_noise[self.noiseIndex][6]
            self.noiseIndex += 1
        else:
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

        self.noisy_pose_pub.publish(pose_stamped)
    
    def generateDriftSignal(self):
        """ generate a whole drift signal that can be saved in a file and be used later """
        self.generatePoseNoise()
        self.total_noise = self.pose_noise 

    def generatePoseNoise(self):
        """ Generate the additive noise which is irrelevant of the robot's motion """
        sigma = np.diag([self.pose_noise_amplitude] * 6)
        generated_noise = np.random.randn(self.max_noise_length, 6) # white gaussian noise
        self.pose_noise = np.clip(generated_noise, -self.pose_noise_amplitude, self.pose_noise_amplitude)
    
    def generateVelocityNoise(self, current_velocity: np.array):
        """ Generative multiplicative noise which is relevant of the current speed """
        # TODO(supernlogn)
        self.velocityNoise = []

    def saveNoiseToFile(self, file_path: Path):
        """ save noise generated data to a file """
        assert(not(file_path.is_dir())), f"{file_path}: please provide a file not a directory"
        with open(str(file_path), "w") as fw:
            json.dump({
                "max_noise_length": self.max_noise_length,
                "pose_noise_amplitude": self.pose_noise_amplitude,
                "poseNoise": list(np.reshape(self.pose_noise, [-1])),
                "velocityNoise": 0
            },fw)

    def loadNoiseFromFile(self, file_path: Path):
        """ load noisy data from a file """
        if(file_path.exists()):
            loadedData:dict = json.load(str(file_path))
            self.max_noise_length = loadedData.get("max_noise_length")
            self.pose_noise_amplitude = loadedData.get("pose_noise_amplitude")
            self.pose_noise = loadedData.get("poseNoise")
            self.velocityNoise = loadedData.get("velocityNoise")


def mainGenerator(args):
    dG = DriftGenerator(save_file_path="temp.json", pose_noise_amplitude=0.1)

def plotPathAndNoise():
    pass

def main(_):
    rospy.init_node('flat_data_player')
    dG = DriftGenerator(load_file_path="temp.json", 
                        load_from_file=True, 
                        save_to_file=False, 
                        use_as_generator=False)
    rospy.spin()

if __name__ == "__main__":
    main(sys.argv)

# From https://github.com/ethz-asl/unreal_airsim/blob/master/src/simulator_processing/odometry_drift_simulator/odometry_drift_simulator.cpp
#   // Noise distributions
#   struct VelocityNoiseDistributions {
#     explicit VelocityNoiseDistributions(
#         const Config::NoiseConfigMap& velocity_noise_configs);
#     NoiseDistribution x, y, z;
#     NoiseDistribution yaw;
#   } velocity_noise_;
#   struct PoseNoiseDistributions {
#     explicit PoseNoiseDistributions(
#         const Config::NoiseConfigMap& pose_noise_configs);
#     NoiseDistribution x, y, z;
#     NoiseDistribution yaw, pitch, roll;
#   } pose_noise_;
# void OdometryDriftSimulator::tick(
#     const geometry_msgs::TransformStamped& ground_truth_pose_msg) {
#   // Compute the time delta
#   const ros::Time& current_timestamp = ground_truth_pose_msg.header.stamp;
#   double delta_t = 0.0;
#   if (!last_ground_truth_pose_msg_.header.stamp.isZero()) {
#     delta_t =
#         (current_timestamp - last_ground_truth_pose_msg_.header.stamp).toSec();
#   }
#   last_ground_truth_pose_msg_ = ground_truth_pose_msg;
#   if (delta_t < 0.0) {
#     LOG(WARNING) << "Time difference between current and last received pose "
#                     "msg is negative. Skipping.";
#     return;
#   }

#   // Get the ground truth pose
#   Transformation ground_truth_pose;
#   tf::transformMsgToKindr(ground_truth_pose_msg.transform, &ground_truth_pose);

#   // Draw a random velocity noise sample, used to simulate drift
#   if (last_velocity_noise_sampling_time_ + velocity_noise_sampling_period_ <
#       current_timestamp) {
#     last_velocity_noise_sampling_time_ = current_timestamp;

#     // Sample the linear velocity noise in body frame
#     const Transformation::Vector3 current_linear_velocity_noise_sample_B_ = {
#         velocity_noise_.x(), velocity_noise_.y(), velocity_noise_.z()};
#     current_linear_velocity_noise_sample_W_ =
#         ground_truth_pose.getRotation().rotate(
#             current_linear_velocity_noise_sample_B_);

#     // Sample the angular velocity noise directly in world frame,
#     // since we only want to simulate drift on world frame yaw
#     current_angular_velocity_noise_sample_W_ = {0.0, 0.0,
#                                                 velocity_noise_.yaw()};
#   }

#   // Integrate the drift
#   Transformation::Vector6 drift_delta_W_vec;
#   drift_delta_W_vec << current_linear_velocity_noise_sample_W_,
#       current_angular_velocity_noise_sample_W_;
#   drift_delta_W_vec *= delta_t;
#   const Transformation drift_delta_W = Transformation::exp(drift_delta_W_vec);
#   integrated_pose_drift_ = drift_delta_W * integrated_pose_drift_;

#   // Draw a random pose noise sample
#   Transformation::Vector6 pose_noise_B_vec;
#   pose_noise_B_vec << pose_noise_.x(), pose_noise_.y(), pose_noise_.z(),
#       pose_noise_.roll(), pose_noise_.pitch(), pose_noise_.yaw();
#   current_pose_noise_ = Transformation::exp(pose_noise_B_vec);

#   // Update the current simulated pose
#   current_simulated_pose_ =
#       integrated_pose_drift_ * ground_truth_pose * current_pose_noise_;
# }