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

  // Return a sample from the normal distribution N(mean_, stddev_)
  double operator()() {
    if(noise_values.empty()) {
      initializeValues(noise_file_path_);
    }
    std::string new_num_str = "";
    // get a number sample from the file
    static int values_size = noise_values.size();
    int real_index = index % values_size;
    const double random_num = noise_values[real_index];
    ++index;
    return random_num;
  }
  static std::shared_ptr<NormalDistribution> getNormalDistribution(const std::string noise_file_path="") {
    if(normalDistribution == nullptr) {
      normalDistribution = std::make_shared<NormalDistribution>(noise_file_path); 
    }
    return normalDistribution;
  }

  static std::shared_ptr<NormalDistribution> getNormalDistribution(const Config& config) {
    if(normalDistribution == nullptr) {
      normalDistribution = std::make_shared<NormalDistribution>(config); 
    }
    return normalDistribution;
  }
  NormalDistribution(const NormalDistribution &other) = default;
  static std::shared_ptr<NormalDistribution> normalDistribution;
 private:
  NormalDistribution(std::string noise_file_path)
      : noise_file_path_(noise_file_path) {
      LOG(WARNING) << "opening " << noise_file_path_;
    if(noise_values.empty()) {
      NormalDistribution::initializeValues(noise_file_path);
    }
    // CHECK(noise_stream_->good());
  }
  NormalDistribution(const Config& config)
      : NormalDistribution(config.noise_file_path) {}
  NormalDistribution() {}
  void initializeValues(std::string file_path) {
    std::ifstream noisy_stream(file_path);
    std::string new_num_str = "";
    while(std::getline(noisy_stream, new_num_str, ',')) {
      double num = std::stod(new_num_str);
      noise_values.push_back(num);
    }
    index = 0;
  }
  // const double mean_, stddev_;
  const std::string noise_file_path_;
  std::vector<double> noise_values;
  int index;
  // Standard normal distribution
  // std::normal_distribution<double> normal_distribution_;
};
}  // namespace unreal_airsim

#endif  // UNREAL_AIRSIM_SIMULATOR_PROCESSING_ODOMETRY_DRIFT_SIMULATOR_NORMAL_DISTRIBUTION_H_
