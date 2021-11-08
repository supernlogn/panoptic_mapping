#include <glog/logging.h>
#include <ros/ros.h>
#include <vector>
#include <csignal>
#include <memory>

#include "panoptic_mapping_utils/drift_generator/drift_generator.h"
// Lets the simulator shutdown in a controlled fashion
std::unique_ptr<DriftGenerator> the_generator = nullptr;
void sigintHandler(int sig) {
  if (the_generator) {
    the_generator->onShutdown();
  }
  ros::shutdown();
}


int main(int argc, char ** argv) {
  ros::init(argc, argv, "drift_generator", ros::init_options::NoSigintHandler);
  // Setup logging
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::ParseCommandLineFlags(&argc, &argv, false);

  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");
  signal(SIGINT, sigintHandler);
  the_generator =
      std::make_unique<DriftGenerator>(nh, nh_private);
  int num_threads = 1;
  ros::AsyncSpinner spinner(num_threads);
  spinner.start();
  ros::waitForShutdown();
  return 0;
}