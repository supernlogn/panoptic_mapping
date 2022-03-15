#include <panoptic_mapping/3rd_party/config_utilities.hpp>
#include <ros/ros.h>

#include "panoptic_mapping_utils/translation/panmap_translator.h"

int main(int argc, char** argv) {
  config_utilities::RequiredArguments ra(
      &argc, &argv, {"--logtostderr", "--colorlogtostderr"});

  // Setup logging.
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::ParseCommandLineFlags(&argc, &argv, false);

  // Run ros.
  ros::init(argc, argv, "evaluation_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Evaluator.
  panoptic_mapping::PanmapTranslator translator(nh, nh_private);
  ros::spin();
  return 0;
}
