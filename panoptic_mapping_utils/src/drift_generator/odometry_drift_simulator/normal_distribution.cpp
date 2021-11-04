#include "panoptic_mapping_utils/drift_generator/odometry_drift_simulator/normal_distribution.h"

#include <string>

namespace unreal_airsim {
NormalDistribution::Config NormalDistribution::Config::fromRosParams(
    const ros::NodeHandle& nh) {
  Config config;
  nh.param<std::string>("noise_file_path", config.noise_file_path);
  return config;
}

bool NormalDistribution::Config::isValid(
    const std::string& error_msg_prefix) const {
    bool ok = std::ifstream(noise_file_path).good();
  if (!ok) {
    LOG_IF(WARNING, !error_msg_prefix.empty())
        << "The " << error_msg_prefix
        << "/file_path:" << noise_file_path << " does not exist";
    return false;
  }
  return true;
}

std::ostream& operator<<(std::ostream& os,
                         const NormalDistribution::Config& config) {
  return os << "noise_file_path: " << config.noise_file_path;
}
}  // namespace unreal_airsim
