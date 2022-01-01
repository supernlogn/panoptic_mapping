#include "panoptic_mapping_utils/drift_generator/drift_generator.h"

#include <fstream>
#include <vector>

#include <glog/logging.h>
#include <ros/ros.h>

#include "panoptic_mapping_utils/drift_generator/odometry_drift_simulator/odometry_drift_simulator.h"

DriftGenerator::DriftGenerator(const ros::NodeHandle& nh,
                               const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      noise_file_output_(initializeStreamFromRosParams(
          DriftGenerator::Config::fromRosParams(nh_private))),
      odometry_drift_simulator_(
          unreal_airsim::OdometryDriftSimulator::Config::fromRosParams(
              nh_private)) {
  readParamsFromRos();
  setupROS();
}

void DriftGenerator::generate_noisy_pose_callback(
    const geometry_msgs::TransformStamped& msg) {
  odometry_drift_simulator_.tick(msg);
  geometry_msgs::TransformStamped msg_noisy_pose =
      odometry_drift_simulator_.getSimulatedPoseMsg();
  this->noisy_pose_pub_.publish(msg_noisy_pose);
  msg_noisy_pose.child_frame_id = config_.sensor_frame_name;
  msg_noisy_pose.header.frame_id = config_.global_frame_name;
  msg_noisy_pose.header.stamp = msg.header.stamp;
  noisy_transform_broadcaster_.sendTransform(msg_noisy_pose);
  // save a python dictionary with the original pose and the pose with added
  // noise.

  if (config_.save_to_file) {
    noise_file_output_ << "{"
                       << "\"time\": " << msg.header.stamp.toSec() << ", ";
    noise_file_output_ << "\"original\": {";
    noise_file_output_ << "\"rotation\""
                       << ": [" << msg.transform.rotation.x << ", "
                       << msg.transform.rotation.y << ", "
                       << msg.transform.rotation.z << ", "
                       << msg.transform.rotation.w << "], ";
    noise_file_output_ << "\"translation\""
                       << ": [" << msg.transform.translation.x << ", "
                       << msg.transform.translation.y << ", "
                       << msg.transform.translation.z << "]";
    noise_file_output_ << "}, ";
    noise_file_output_ << "\"noisy\":{";
    noise_file_output_ << "\"rotation\""
                       << ": [" << msg_noisy_pose.transform.rotation.x << ", "
                       << msg_noisy_pose.transform.rotation.y << ", "
                       << msg_noisy_pose.transform.rotation.z << ", "
                       << msg_noisy_pose.transform.rotation.w << "], ";
    noise_file_output_ << "\"translation\""
                       << ": [" << msg_noisy_pose.transform.translation.x
                       << ", " << msg_noisy_pose.transform.translation.y << ", "
                       << msg_noisy_pose.transform.translation.z << "]";
    noise_file_output_ << "}";
    noise_file_output_ << "}" << '\n';
  }
}

void DriftGenerator::startupCallback(const ros::TimerEvent&) {
  // odometry_drift_simulator_.start();
  LOG(INFO) << "DriftGenerator is ready!";
}

void DriftGenerator::onShutdown() {
  LOG(INFO) << "Shutting down: resetting DriftGenerator.";
}

bool DriftGenerator::setupROS() {
  // int queue_length = 50;
  // if(config_.save_to_file) {
  int queue_length = 100;
  // }
  noisy_pose_pub_ = nh_.advertise<geometry_msgs::TransformStamped>(
      config_.noisy_pose_topic, queue_length);
  pose_sub_ =
      nh_.subscribe(config_.ground_truth_pose_topic, queue_length,
                    &DriftGenerator::generate_noisy_pose_callback, this);
  return true;
}

bool DriftGenerator::readParamsFromRos() {
  DriftGenerator::Config defaults;
  nh_private_.param("save_noise_file_path", config_.save_file_path,
                    defaults.save_file_path);
  nh_private_.param("save_noise_to_file", config_.save_to_file,
                    defaults.save_to_file);
  nh_private_.param("global_frame_name", config_.global_frame_name,
                    defaults.global_frame_name);
  nh_private_.param("sensor_frame_name", config_.sensor_frame_name,
                    defaults.sensor_frame_name);
  nh_private_.param("noisy_pose_topic", config_.noisy_pose_topic,
                    defaults.noisy_pose_topic);
  nh_private_.param("ground_truth_pose_topic", config_.ground_truth_pose_topic,
                    defaults.ground_truth_pose_topic);
  return true;
}

DriftGenerator::Config DriftGenerator::Config::fromRosParams(
    const ros::NodeHandle& nh) {
  DriftGenerator::Config cfg;
  DriftGenerator::Config defaults;
  nh.param("save_noise_file_path", cfg.save_file_path, defaults.save_file_path);
  nh.param("save_noise_to_file", cfg.save_to_file, defaults.save_to_file);
  nh.param("global_frame_name", cfg.global_frame_name,
           defaults.global_frame_name);
  nh.param("sensor_frame_name", cfg.sensor_frame_name,
           defaults.sensor_frame_name);
  nh.param("noisy_pose_topic", cfg.noisy_pose_topic, defaults.noisy_pose_topic);
  nh.param("ground_truth_pose_topic", cfg.ground_truth_pose_topic,
           defaults.ground_truth_pose_topic);
  return cfg;
}
