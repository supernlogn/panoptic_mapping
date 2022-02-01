#include "panoptic_mapping_utils/drift_generator/drift_generator.h"

#include <fstream>
#include <vector>

#include <glog/logging.h>
#include <ros/ros.h>
#include <tf/tfMessage.h>

#include "panoptic_mapping_utils/drift_generator/odometry_drift_simulator/normal_distribution.h"
#include "panoptic_mapping_utils/drift_generator/odometry_drift_simulator/odometry_drift_simulator.h"

#include "voxgraph/tools/io.h"

DriftGenerator::DriftGenerator(const ros::NodeHandle& nh,
                               const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
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
  msg_noisy_pose.child_frame_id = config_.sensor_frame_name;
  msg_noisy_pose.header.frame_id = config_.global_frame_name;
  if (msg.header.stamp < ros::TIME_MIN) {
    msg_noisy_pose.header.stamp = ros::TIME_MIN;
  } else {
    msg_noisy_pose.header.stamp = msg.header.stamp;
  }
  if (config_.use_tf_transforms) {
    noisy_transform_broadcaster_.sendTransform(msg_noisy_pose);
  } else {
    noisy_pose_pub_.publish(msg_noisy_pose);
  }
  geometry_msgs::TransformStamped ground_truth_msg;
  ground_truth_msg.child_frame_id = config_.sensor_frame_name;
  ground_truth_msg.header = msg.header;
  ground_truth_msg.transform = msg.transform;
  // store poses to two vectors to save them later to a bag
  // file.
  ground_truth_poses_.push_back(ground_truth_msg);
  noisy_poses_.push_back(msg_noisy_pose);
}

void DriftGenerator::startupCallback(const ros::TimerEvent&) {
  // odometry_drift_simulator_.start();
  LOG(INFO) << "DriftGenerator is ready!";
}

void DriftGenerator::onShutdown() {
  // Write the poses generated to a rosbag
  rosbag::Bag bag;
  bag.open(config_.save_file_path, rosbag::bagmode::Write);
  for (geometry_msgs::TransformStamped tf_stamped : ground_truth_poses_) {
    if (tf_stamped.header.stamp < ros::TIME_MIN) {
      tf_stamped.header.stamp = ros::TIME_MIN;
    }
    bag.write(config_.ground_truth_pose_topic, tf_stamped.header.stamp,
              tf_stamped);
  }
  for (const geometry_msgs::TransformStamped& tf_stamped : noisy_poses_) {
    bag.write(config_.noisy_pose_topic, tf_stamped.header.stamp, tf_stamped);
  }
  // write to /tf
  for (const geometry_msgs::TransformStamped& tf_stamped : noisy_poses_) {
    tf::tfMessage tf_msg;
    tf_msg.transforms =
        std::vector<geometry_msgs::TransformStamped>{tf_stamped};
    bag.write("/tf", tf_stamped.header.stamp, tf_msg);
  }
  bag.close();
  // inform for closing
  LOG(INFO) << "Shutting down: resetting DriftGenerator.";
}

bool DriftGenerator::setupROS() {
  const int queue_length = 100;
  noisy_pose_pub_ = nh_.advertise<geometry_msgs::TransformStamped>(
      config_.noisy_pose_topic, queue_length);
  pose_sub_ =
      nh_.subscribe(config_.ground_truth_pose_topic, queue_length,
                    &DriftGenerator::generate_noisy_pose_callback, this);
  return true;
}

bool DriftGenerator::readParamsFromRos() {
  DriftGenerator::Config defaults;
  nh_private_.param("generated_path_file_path", config_.save_file_path,
                    defaults.save_file_path);
  nh_private_.param("global_frame_name", config_.global_frame_name,
                    defaults.global_frame_name);
  nh_private_.param("sensor_frame_name", config_.sensor_frame_name,
                    defaults.sensor_frame_name);
  nh_private_.param("noisy_pose_topic", config_.noisy_pose_topic,
                    defaults.noisy_pose_topic);
  nh_private_.param("ground_truth_pose_topic", config_.ground_truth_pose_topic,
                    defaults.ground_truth_pose_topic);
  nh_private_.param("use_tf_transforms", config_.use_tf_transforms,
                    defaults.use_tf_transforms);
  return true;
}

DriftGenerator::Config DriftGenerator::Config::fromRosParams(
    const ros::NodeHandle& nh) {
  DriftGenerator::Config cfg;
  DriftGenerator::Config defaults;
  nh.param("generated_path_file_path", cfg.save_file_path,
           defaults.save_file_path);
  nh.param("global_frame_name", cfg.global_frame_name,
           defaults.global_frame_name);
  nh.param("sensor_frame_name", cfg.sensor_frame_name,
           defaults.sensor_frame_name);
  nh.param("noisy_pose_topic", cfg.noisy_pose_topic, defaults.noisy_pose_topic);
  nh.param("ground_truth_pose_topic", cfg.ground_truth_pose_topic,
           defaults.ground_truth_pose_topic);
  nh.param("use_tf_transforms", cfg.use_tf_transforms,
           defaults.use_tf_transforms);
  return cfg;
}
