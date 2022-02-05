#ifndef PANOPTIC_MAPPING_UTILS_DRIFT_GENERATOR_DRIFT_GENERATOR_H_
#define PANOPTIC_MAPPING_UTILS_DRIFT_GENERATOR_DRIFT_GENERATOR_H_

#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "odometry_drift_simulator/odometry_drift_simulator.h"

class DriftGenerator {
 public:
  struct Config {
    // Initialize from ROS params
    static Config fromRosParams(const ros::NodeHandle& nh);
    std::string save_file_path;
    std::string noisy_pose_topic;
    std::string ground_truth_pose_topic;
    std::string sensor_frame_name = "depth_camera";
    std::string global_frame_name = "world";
    bool use_tf_transforms = true;
  };

  explicit DriftGenerator(const ros::NodeHandle& nh,
                          const ros::NodeHandle& nh_private);
  ~DriftGenerator() = default;

  // ROS callbacks
  void generate_noisy_pose_callback(const geometry_msgs::TransformStamped& msg);
  void startupCallback(const ros::TimerEvent&);
  void onShutdown();  // called by the sigint handler
  static std::ofstream initializeStreamFromRosParams(Config cfg) {
    if (!cfg.save_file_path.empty()) {
      std::ofstream outstream(cfg.save_file_path.c_str(), std::ofstream::out);
      return outstream;
    }
    return std::ofstream("no_text.txt", std::ofstream::app);
  }

 private:
  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher noisy_pose_pub_;
  ros::Subscriber pose_sub_;
  std::vector<geometry_msgs::TransformStamped> noisy_poses_;
  std::vector<geometry_msgs::TransformStamped> ground_truth_poses_;
  tf2_ros::TransformBroadcaster noisy_transform_broadcaster_;
  bool setupROS();
  bool readParamsFromRos();
  // simulator
  unreal_airsim::OdometryDriftSimulator odometry_drift_simulator_;
  // tools
  Config config_;
  // constants
  const std::string simulator_frame_name_ = "noisy_frame";
};

#endif  // PANOPTIC_MAPPING_UTILS_DRIFT_GENERATOR_DRIFT_GENERATOR_H_
