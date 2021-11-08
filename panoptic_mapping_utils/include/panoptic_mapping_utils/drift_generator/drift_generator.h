#include "odometry_drift_simulator/odometry_drift_simulator.h"
#include <memory>
#include <string>


#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>



class DriftGenerator {
 public:
    struct Config {
        // Initialize from ROS params
        static Config fromRosParams(const ros::NodeHandle& nh);
        std::string noise_file_path;
        std::string save_file_path;
        bool save_to_file;
        std::string noisy_pose_topic;
        std::string ground_truth_Pose_topic;
        // Write config values to stream, e.g. for logging
        // friend std::ostream& operator<<(std::ostream& os, const Config& config);
    };
    
    explicit DriftGenerator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    ~DriftGenerator() = default;

  // ROS callbacks
  void generate_noisy_pose_callback(const geometry_msgs::TransformStamped& msg);
  void startupCallback(const ros::TimerEvent&);
  void onShutdown();  // called by the sigint handler

 private:
   // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher noisy_pose_pub_;
  ros::Subscriber pose_sub_;
  bool setupROS();
  bool readParamsFromRos();
  // simulator
  unreal_airsim::OdometryDriftSimulator odometry_drift_simulator_;
  // tools
  Config config_;
  // constants
  const std::string simulator_frame_name_ = "noisy_frame";

};