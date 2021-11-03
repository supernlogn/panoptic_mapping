#include "panoptic_mapping/tracking/ground_truth_id_tracker.h"

#include <memory>
#include <unordered_set>
#include <utility>
#include <vector>

#include <minkindr_conversions/kindr_tf.h>
#include <minkindr_conversions/kindr_msg.h>
#include <voxblox_ros/conversions.h>

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<IDTrackerBase, GroundTruthIDTracker,
                                           std::shared_ptr<Globals>>
    GroundTruthIDTracker::registration_("ground_truth");

void GroundTruthIDTracker::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
}

GroundTruthIDTracker::GroundTruthIDTracker(const Config& config,
                                           std::shared_ptr<Globals> globals,
                                           bool print_config)
    : config_(config.checkValid()), IDTrackerBase(std::move(globals)) {
  LOG_IF(INFO, config_.verbosity >= 1 && print_config) << "\n"
                                                       << config_.toString();
  addRequiredInputs({InputData::InputType::kSegmentationImage,
                     InputData::InputType::kValidityImage});
  background_submap_publisher = nh_.advertise<voxblox_msgs::Submap>(background_submap_topic_name_, 100);
}

void GroundTruthIDTracker::processInput(SubmapCollection* submaps,
                                        InputData* input) {
  CHECK_NOTNULL(submaps);
  CHECK_NOTNULL(input);
  CHECK(inputIsValid(*input));
  // Look for new instances that are within integration range.
  std::unordered_set<int> instances;
  for (int u = 0; u < input->idImage().cols; ++u) {
    for (int v = 0; v < input->idImage().rows; ++v) {
      if (input->validityImage().at<uchar>(v, u)) {
        instances.insert(input->idImage().at<int>(v, u));
      }
    }
  }

  // Allocate new submaps if necessary.
  for (const int instance : instances) {
    parseInputInstance(instance, submaps, input);
  }
  printAndResetWarnings();

  // Set segmentation image to submap ids.
  for (auto it = input->idImagePtr()->begin<int>();
       it != input->idImagePtr()->end<int>(); ++it) {
    auto it2 = instance_to_id_.find(*it);
    if (it2 == instance_to_id_.end()) {
      *it = -1;
    } else {
      *it = it2->second;
    }
  }

  // Allocate free space map if required.
  freespace_allocator_->allocateSubmap(submaps, input);
}

bool GroundTruthIDTracker::parseInputInstance(int instance,
                                              SubmapCollection* submaps,
                                              InputData* input) {
  // Known existing submap.
  auto it = instance_to_id_.find(instance);
  if (it != instance_to_id_.end()) {
    if (submaps->submapIdExists(it->second)) {
      submaps->getSubmapPtr(it->second)->setWasTracked(true);
      return true;
    } else {
      LOG_IF(WARNING, config_.verbosity >= 2)
          << "Submap '" << it->second << "' for instance ID '" << instance
          << "' has been deleted.";
      instance_to_id_.erase(it);
    }
  }

  // Check whether the instance code is known.
  if (!globals_->labelHandler()->segmentationIdExists(instance)) {
    auto error_it = unknown_ids.find(instance);
    if (error_it == unknown_ids.end()) {
      unknown_ids[instance] = 1;
    } else {
      error_it->second++;
    }
    return false;
  }

  // Allocate new submap.
  const panoptic_mapping::LabelEntry & submapLabel = globals_->labelHandler()->getLabelEntry(instance);
  Submap* new_submap = submap_allocator_->allocateSubmap(
      submaps, input, instance,
      submapLabel);
  if (new_submap) {
    new_submap->setInstanceID(instance);
    instance_to_id_[instance] = new_submap->getID();
    // if it is a background submap, send it to voxgraph
    if(new_submap->getLabel() == PanopticLabel::kBackground) {
      publishSubmapToVoxGraph(*new_submap);
    }

    return true;
  } else {
    LOG_IF(WARNING, config_.verbosity >= 2)
        << "Submap allocation failed for input ID '" << instance << "'.";
    return false;
  }
}

void GroundTruthIDTracker::printAndResetWarnings() {
  if (config_.verbosity < 2) {
    unknown_ids.clear();
    return;
  }
  for (auto it : unknown_ids) {
    LOG(WARNING) << "Encountered " << it.second
                 << " occurences of unknown segmentation ID '" << it.first
                 << "'.";
  }
  unknown_ids.clear();
}


void GroundTruthIDTracker::publishSubmapToVoxGraph(Submap & submapToPublish) {
  // This code is from Voxblox::TsdfServer::publishSubmap
  // assume that submapToPublish is not null
  voxblox_msgs::Submap submap_msg;
  submap_msg.robot_name = "robot";
  
  voxblox::serializeLayerAsMsg<TsdfVoxel>(submapToPublish.getTsdfLayer(),
                                   /* only_updated */ false, &submap_msg.layer);  
  // submap_msg.trajectory
  background_submap_publisher.publish(submap_msg);
}


}  // namespace panoptic_mapping
