#include <panoptic_mapping/3rd_party/config_utilities.hpp>
#include <ros/ros.h>

#include "panoptic_mapping_utils/evaluation/trajectory_evaluator.h"

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
  panoptic_mapping::TrajectoryEvaluator evaluator(nh, nh_private);

  // Process the request
  auto request = config_utilities::getConfigFromRos<
      panoptic_mapping::TrajectoryEvaluator::EvaluationRequest>(nh_private);
  evaluator.evaluate(request);

  // Visualize.
  return 0;
}
