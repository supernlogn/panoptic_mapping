#ifndef PANOPTIC_MAPPING_UTILS_TRANSLATION_PANMAP_TRANSLATOR_H_
#define PANOPTIC_MAPPING_UTILS_TRANSLATION_PANMAP_TRANSLATOR_H_

#include <memory>
#include <string>
#include <utility>

#include <panoptic_mapping/3rd_party/config_utilities.hpp>
#include <panoptic_mapping/3rd_party/nanoflann.hpp>
#include <panoptic_mapping/common/common.h>
#include <panoptic_mapping/map/submap_collection.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

namespace panoptic_mapping {

using pclPointType = pcl::PointXYZ;

class PanmapTranslator {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;

    // Data handling.
    std::string map_file = "";
    std::string output_pointcloud_file = "";
    std::string output_directory = "";
    std::string output_postfix = ".ply";

    bool visualize = false;
    bool binary = false;

    Config() { setConfigName("PanmapTranslator::Config"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  // constructor
  PanmapTranslator(const ros::NodeHandle& nh,
                   const ros::NodeHandle& nh_private);
  virtual ~PanmapTranslator() = default;

  // translate the file
  void translate();
  void saveToPCLFile() const;

 private:
  bool readSubmaps();

 private:
  // ROS.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Files.
  std::ofstream output_file_;

  // Stored data.
  std::unique_ptr<pcl::PointCloud<pclPointType>> cloud_buffer_;
  std::shared_ptr<SubmapCollection> submaps_;
  // config
  Config config_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_UTILS_TRANSLATION_PANMAP_TRANSLATOR_H_
