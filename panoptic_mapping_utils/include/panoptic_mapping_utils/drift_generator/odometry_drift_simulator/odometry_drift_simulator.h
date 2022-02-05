#ifndef PANOPTIC_MAPPING_UTILS_DRIFT_GENERATOR_ODOMETRY_DRIFT_SIMULATOR_ODOMETRY_DRIFT_SIMULATOR_H_
#define PANOPTIC_MAPPING_UTILS_DRIFT_GENERATOR_ODOMETRY_DRIFT_SIMULATOR_ODOMETRY_DRIFT_SIMULATOR_H_

#include <map>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <kindr/minimal/quat-transformation.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include "normal_distribution.h"  // NOLINT

namespace unreal_airsim {
class OdometryDriftSimulator {
 public:
  using NoiseDistribution = NormalDistribution;
  using Transformation = kindr::minimal::QuatTransformationTemplate<double>;

  struct Config {
    // Initialize from ROS params
    static Config fromRosParams(const ros::NodeHandle& nh);

    // Whether and how to publish the ground truth pose
    bool publish_ground_truth_pose = false;
    std::string ground_truth_frame_suffix = "_ground_truth";

    // Params of the distributions used to generate the pose drift and noise
    float velocity_noise_frequency_hz = 1;
    int noise_config_seed = 2;
    using NoiseConfigMap = std::map<std::string, NoiseDistribution::Config>;
    NoiseConfigMap velocity_noise = {
        {"x", {}}, {"y", {}}, {"z", {}}, {"yaw", {}}};
    NoiseConfigMap pose_noise = {{"x", {}},   {"y", {}},     {"z", {}},
                                 {"yaw", {}}, {"pitch", {}}, {"roll", {}}};

    // Validity queries and assertions
    bool isValid() const;
    Config& checkValid() {
      CHECK(isValid());
      return *this;
    }

    // Write config values to stream, e.g. for logging
    friend std::ostream& operator<<(std::ostream& os, const Config& config);
  };

  explicit OdometryDriftSimulator(Config config);
  ~OdometryDriftSimulator() = default;

  void start() {
    reset();
    started_publishing_ = true;
  }
  void reset();
  void tick(const geometry_msgs::TransformStamped& ground_truth_pose_msg);

  Transformation getSimulatedPose() const { return current_simulated_pose_; }
  Transformation getGroundTruthPose() const;
  geometry_msgs::TransformStamped getSimulatedPoseMsg() const;
  geometry_msgs::TransformStamped getGroundTruthPoseMsg() const {
    return last_ground_truth_pose_msg_;
  }

  Transformation convertDriftedToGroundTruthPose(
      const Transformation& simulated_pose) const;
  Transformation convertGroundTruthToDriftedPose(
      const Transformation& ground_truth_pose) const;
  geometry_msgs::TransformStamped convertDriftedToGroundTruthPoseMsg(
      const geometry_msgs::TransformStamped& simulated_pose_msg) const;
  geometry_msgs::TransformStamped convertGroundTruthToDriftedPoseMsg(
      const geometry_msgs::TransformStamped& ground_truth_pose_msg) const;
  void publishTfs() const;

 private:
  // Settings
  const Config config_;
  bool started_publishing_;

  // Simulator state
  ros::Time last_velocity_noise_sampling_time_;
  const ros::Duration velocity_noise_sampling_period_;
  Transformation::Vector3 current_linear_velocity_noise_sample_W_;
  Transformation::Vector3 current_angular_velocity_noise_sample_W_;
  Transformation integrated_pose_drift_;
  Transformation current_pose_noise_;
  Transformation current_simulated_pose_;
  Transformation current_absolute_noise_pose_;
  geometry_msgs::TransformStamped last_ground_truth_pose_msg_;

  // Noise distributions
  struct VelocityNoiseDistributions {
    explicit VelocityNoiseDistributions(
        const Config::NoiseConfigMap& velocity_noise_configs);
    NoiseDistribution x, y, z;
    NoiseDistribution yaw;
  } velocity_noise_;
  struct PoseNoiseDistributions {
    explicit PoseNoiseDistributions(
        const Config::NoiseConfigMap& pose_noise_configs);
    NoiseDistribution x, y, z;
    NoiseDistribution yaw, pitch, roll;
  } pose_noise_;

  // Transform publishing
  mutable tf2_ros::TransformBroadcaster transform_broadcaster_;
  void publishSimulatedPoseTf() const;
};
}  // namespace unreal_airsim

#endif  // PANOPTIC_MAPPING_UTILS_DRIFT_GENERATOR_ODOMETRY_DRIFT_SIMULATOR_ODOMETRY_DRIFT_SIMULATOR_H_
