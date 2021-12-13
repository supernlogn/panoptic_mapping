#include "panoptic_mapping/tracking/ground_truth_id_tracker.h"

#include <memory>
#include <unordered_set>
#include <utility>
#include <vector>

#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>

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
  LabelEntry label;
  const bool label_exists = getLabelIfExists(instance, &label);
  const bool is_2new_background =
      (label_exists && label.label == PanopticLabel::kBackground &&
       submaps->backgroundExists());
  // Known existing submap.
  if (is_2new_background) {
    Submap* submap = submaps->getBackground();
    submap->setWasTracked(true);
    return true;
  } else {
    auto it = instance_to_id_.find(instance);
    if (it != instance_to_id_.end()) {
      if (submaps->submapIdExists(it->second)) {
        Submap* submap = submaps->getSubmapPtr(it->second);
        submap->setWasTracked(true);
        return true;
      } else {
        LOG_IF(WARNING, config_.verbosity >= 2)
            << "Submap '" << it->second << "' for instance ID '" << instance
            << "' has been deleted.";
        instance_to_id_.erase(it);
      }
    }
  }
  // Check whether the instance code is known.
  if (!label_exists) {
    auto error_it = unknown_ids.find(instance);
    if (error_it == unknown_ids.end()) {
      unknown_ids[instance] = 1;
    } else {
      error_it->second++;
    }
    return false;
  }

  // Allocate new submap.
  Submap* new_submap =
      submap_allocator_->allocateSubmap(submaps, input, instance, label);
  if (new_submap) {
    new_submap->setInstanceID(instance);
    instance_to_id_[instance] = new_submap->getID();
    if (label.label == PanopticLabel::kBackground) {
      // backgroundExists() == false
      // this can occur only at tracking start
      submaps->setBackgroundID(new_submap->getID());
    }
    return true;
  } else {
    LOG_IF(WARNING, config_.verbosity >= 2)
        << "Submap allocation failed for input ID '" << instance << "'.";
    return false;
  }
}

bool GroundTruthIDTracker::getLabelIfExists(const int instance,
                                            LabelEntry* label) const {
  bool ret = globals_->labelHandler()->segmentationIdExists(instance);
  if (ret) {
    *label = globals_->labelHandler()->getLabelEntry(instance);
  }
  return ret;
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

}  // namespace panoptic_mapping
