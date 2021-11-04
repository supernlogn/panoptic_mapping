#ifndef UNREAL_AIRSIM_SIMULATOR_PROCESSING_ODOMETRY_DRIFT_SIMULATOR_NORMAL_DISTRIBUTION_H_
#define UNREAL_AIRSIM_SIMULATOR_PROCESSING_ODOMETRY_DRIFT_SIMULATOR_NORMAL_DISTRIBUTION_H_

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>

#include <glog/logging.h>

namespace unreal_airsim {
class NormalDistribution {
 public:
  struct Config {
    // Initialize from ROS params
    static Config fromRosParams(const ros::NodeHandle& nh);

    // Distribution parameters
    std::string noise_file_path;

    // Validity queries and assertions
    bool isValid(const std::string& error_msg_prefix = "") const;
    Config& checkValid() {
      CHECK(isValid());
      return *this;
    }

    // Write config values to stream, e.g. for logging
    friend std::ostream& operator<<(std::ostream& os, const Config& config);
  };

  explicit NormalDistribution(std::string noise_file_path)
      : noise_file_path_(noise_file_path) {
    noise_stream_ = std::make_unique<std::ifstream>(noise_file_path_);
    CHECK(noise_stream_->good());
    
  }
  explicit NormalDistribution(const Config& config)
      : NormalDistribution(config.noise_file_path) {}


  // Return a sample from the normal distribution N(mean_, stddev_)
  double operator()() {
    CHECK(noise_stream_->good());
    std::string new_num_str = "";
    // get a number sample from the file
    std::getline(*noise_stream_, new_num_str, ',');
    double random_num = std::stof(new_num_str);

    return random_num;
  }

 private:
  // const double mean_, stddev_;
  const std::string noise_file_path_;
  std::unique_ptr<std::ifstream> noise_stream_;
  // Standard normal distribution
  // std::normal_distribution<double> normal_distribution_;
};
}  // namespace unreal_airsim

#endif  // UNREAL_AIRSIM_SIMULATOR_PROCESSING_ODOMETRY_DRIFT_SIMULATOR_NORMAL_DISTRIBUTION_H_
