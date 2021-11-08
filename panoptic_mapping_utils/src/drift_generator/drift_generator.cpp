#include <glog/logging.h>
#include <ros/ros.h>
#include <vector>

#include "panoptic_mapping_utils/drift_generator/drift_generator.h"
#include "panoptic_mapping_utils/drift_generator/odometry_drift_simulator/odometry_drift_simulator.h"


DriftGenerator::DriftGenerator(const ros::NodeHandle& nh, 
                               const ros::NodeHandle& nh_private) 
      : nh_(nh),
      nh_private_(nh_private),
      odometry_drift_simulator_(unreal_airsim::OdometryDriftSimulator::Config::fromRosParams(nh_private)){
      
      readParamsFromRos();
      setupROS();
}

void DriftGenerator::generate_noisy_pose_callback(const geometry_msgs::TransformStamped& msg) {
  odometry_drift_simulator_.tick(msg);
  geometry_msgs::TransformStamped msg_noisy_pose = odometry_drift_simulator_.getSimulatedPoseMsg();
  this->noisy_pose_pub_.publish(msg_noisy_pose);
}

void DriftGenerator::startupCallback(const ros::TimerEvent&) {
  // odometry_drift_simulator_.start();
    LOG(INFO) << "DriftGenerator is ready!";
}

void DriftGenerator::onShutdown() {
   LOG(INFO) << "Shutting down: resetting DriftGenerator.";
}


bool DriftGenerator::setupROS() {
  noisy_pose_pub_ = nh_.advertise<geometry_msgs::TransformStamped>(
      config_.noisy_pose_topic, 100);
  
  pose_sub_ = nh_.subscribe(config_.ground_truth_Pose_topic, 10,
                &DriftGenerator::generate_noisy_pose_callback, this);
  return true;
}
bool DriftGenerator::readParamsFromRos() {
  DriftGenerator::Config defaults;
  nh_private_.param("noise_file_path", config_.noise_file_path, defaults.noise_file_path);
  nh_private_.param("save_file_path", config_.save_file_path, defaults.save_file_path);
  nh_private_.param("save_to_file", config_.save_to_file, defaults.save_to_file);
  nh_private_.param("noisy_pose_topic", config_.noisy_pose_topic, defaults.noisy_pose_topic);
  nh_private_.param("ground_truth_Pose_topic", config_.ground_truth_Pose_topic, defaults.ground_truth_Pose_topic);
  return true;   
}