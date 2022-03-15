#include "panoptic_mapping_utils/translation/panmap_translator.h"

#include <filesystem>
#include <memory>
#include <string>
#include <utility>

#include <panoptic_mapping/map/submap_collection.h>
#include <panoptic_mapping_msgs/SaveLoadMap.h>
#include <pcl/io/ply_io.h>

namespace panoptic_mapping {

void PanmapTranslator::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("map_file", &map_file);
  setupParam("output_pointcloud_file", &output_pointcloud_file);
  setupParam("output_directory", &output_directory);
  setupParam("output_postfix", &output_postfix);
  setupParam("visualize", &visualize);
  setupParam("binary", &binary);
}

void PanmapTranslator::Config::checkParams() const {
  checkParamCond(!map_file.empty(), "map_file is empty");
  checkParamCond(std::filesystem::exists(std::filesystem::path{map_file}),
                 "map_file does not exist");
  checkParamCond(!output_pointcloud_file.empty(),
                 "output_pointcloud_file is empty");
  checkParamCond(!output_directory.empty(), "output_directory is empty");
  checkParamCond(
      std::filesystem::exists(std::filesystem::path{output_directory}),
      "directory does not exist");
  checkParamCond(!output_postfix.empty(), "output_postfix is empty");
}

PanmapTranslator::PanmapTranslator(const ros::NodeHandle& nh,
                                   const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  config_ =
      config_utilities::getConfigFromRos<PanmapTranslator::Config>(nh_private_);
  CHECK(readSubmaps());
  translate();
  saveToPCLFile();
  ros::shutdown();
}

void PanmapTranslator::translate() {
  cloud_buffer_ = std::make_unique<pcl::PointCloud<pclPointType>>();
  for (const auto& submap : *submaps_) {
    voxblox::Mesh combined_mesh;
    submap.getMeshLayer().getMesh(&combined_mesh);
    for (const auto& v : combined_mesh.vertices) {
      const pclPointType p(v.x(), v.y(), v.z());
      cloud_buffer_->emplace_back(p);
    }
  }
}

bool PanmapTranslator::readSubmaps() {
  submaps_ = std::make_shared<SubmapCollection>();
  if (!submaps_->loadFromFile(config_.map_file)) {
    LOG(ERROR) << "Could not load panoptic map from '" << config_.map_file
               << "'.";
    submaps_.reset();
    return false;
  }
  return true;
}

void PanmapTranslator::saveToPCLFile() const {
  std::stringstream file_path;
  file_path << config_.output_directory;
  if (config_.output_directory.back() != '/') {
    file_path << "/";
  }
  file_path << config_.output_pointcloud_file;
  if (config_.output_pointcloud_file.find(config_.output_postfix) ==
      config_.output_pointcloud_file.npos) {
    file_path << config_.output_postfix;
  }
  pcl::io::savePLYFile(file_path.str(), *cloud_buffer_, config_.binary);
}

}  // namespace panoptic_mapping
