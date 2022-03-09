#include "panoptic_mapping/integration/multi_class_projective_tsdf_integrator.h"

#include <algorithm>
#include <chrono>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <voxblox/integrator/merge_integration.h>

#include "panoptic_mapping/common/index_getter.h"
#include "panoptic_mapping/map/submap.h"

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<TsdfIntegratorBase,
                                           MultiClassProjectiveIntegrator,
                                           std::shared_ptr<Globals>>
    MultiClassProjectiveIntegrator::registration_("multi_class_projective");

void MultiClassProjectiveIntegrator::Config::checkParams() const {
  checkParamConfig(pi_config);
}

void MultiClassProjectiveIntegrator::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("use_binary_classification", &use_binary_classification);
  setupParam("use_instance_classification", &use_instance_classification);
  setupParam("update_only_tracked_submaps", &update_only_tracked_submaps);
  setupParam("projective_integrator", &pi_config);
  setupParam("id_classes_set_1", &id_classes_set_1);
  setupParam("id_classes_set_2", &id_classes_set_2);
}

MultiClassProjectiveIntegrator::MultiClassProjectiveIntegrator(
    const Config& config, std::shared_ptr<Globals> globals)
    : config_(config.checkValid()),
      ProjectiveIntegrator(config.pi_config, std::move(globals), false) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
  id_classes_set_1_.insert(config.id_classes_set_1.begin(),
                           config.id_classes_set_1.end());
  id_classes_set_2_.insert(config.id_classes_set_2.begin(),
                           config.id_classes_set_2.end());
  // Store class count.
  if (!config_.use_binary_classification &&
      !config_.use_instance_classification) {
    // The +1 is added because 0 is reserved for the belonging submap.
    num_classes_ = globals_->labelHandler()->numberOfLabels() + 1;
  }
}

void MultiClassProjectiveIntegrator::processInput(SubmapCollection* submaps,
                                                  InputData* input) {
  CHECK_NOTNULL(submaps);  // Input is not used here and checked later.
  // Cache submap ids by class.
  submap_id_to_class_.clear();
  for (const Submap& submap : *submaps) {
    submap_id_to_class_.emplace(submap.getID(), submap.getClassID());
  }
  instance_to_active_submap_ids_.clear();
  for (const auto& pair : submaps->getInstanceToSubmapIDTable()) {
    for (const int submap_id : pair.second) {
      // in case the segmentation id has no active submap
      // then use whatever was before
      if (instance_to_active_submap_ids_.count(pair.first) == 0) {
        instance_to_active_submap_ids_.emplace(pair.first, submap_id);
      } else if (submaps->getSubmapPtr(submap_id)->isActive()) {
        instance_to_active_submap_ids_.emplace(pair.first, submap_id);
      }
    }
  }
  const int bckg_id = submaps->getBackgroundID();
  for (const int ids : id_classes_set_1_) {
    instance_to_active_submap_ids_[ids] = bckg_id;
  }
  submap_id_to_class_[-1] = -1;  // Used for unknown classes.
  if (config_.use_instance_classification &&
      !config_.use_binary_classification) {
    // Track the number of classes (where classes in this case are instances).
    // NOTE(schmluk): This is dangerous if submaps are de-allocated and can grow
    // arbitrarily large.
    num_classes_ = submap_id_to_class_.size() + 1;
  }
  // debug
  // const auto & img = input->idImage();
  // int count0s = 0;
  // int count1s = 0;
  // int count2s = 0;
  // for (int i = 0; i < img.rows; ++i) {
  //   for(int j = 0; j < img.cols; ++j) {
  //     count0s += (img.at<int>(i, j) == 0);
  //     count1s += (img.at<int>(i, j) == 1);
  //     count2s += (img.at<int>(i, j) == 2);
  //   }
  // }
  // if (count0s + count1s + count2s > 0) {
  //   LOG(WARNING) << "count0s : " << count0s
  //   << "\ncount1s : " << count1s
  //   << "\ncount2s : " << count2s;
  // }
  // Run the integration.
  ProjectiveIntegrator::processInput(submaps, input);
}

void MultiClassProjectiveIntegrator::updateBlock(
    Submap* submap, InterpolatorBase* interpolator,
    const voxblox::BlockIndex& block_index, const Transformation& T_C_S,
    const InputData& input) const {
  CHECK_NOTNULL(submap);
  // Set up preliminaries.
  if (!submap->getTsdfLayer().hasBlock(block_index)) {
    LOG_IF(WARNING, config_.verbosity >= 1)
        << "Tried to access inexistent block '" << block_index.transpose()
        << "' in submap " << submap->getID() << ".";
    return;
  }
  TsdfBlock& block = submap->getTsdfLayerPtr()->getBlockByIndex(block_index);
  const float voxel_size = block.voxel_size();
  const float truncation_distance = submap->getConfig().truncation_distance;
  const int submap_id = submap->getID();
  const bool is_free_space_submap =
      submap->getLabel() == PanopticLabel::kFreeSpace;
  bool was_updated = false;

  // Allocate the class block if not yet existent and get it.
  ClassBlock::Ptr class_block;
  if (submap->hasClassLayer() &&
      (!config_.update_only_tracked_submaps || submap->wasTracked())) {
    class_block =
        submap->getClassLayerPtr()->allocateBlockPtrByIndex(block_index);
  }

  // Update all voxels.
  for (size_t i = 0; i < block.num_voxels(); ++i) {
    TsdfVoxel& voxel = block.getVoxelByLinearIndex(i);
    const Point p_C = T_C_S * block.computeCoordinatesFromLinearIndex(
                                  i);  // Voxel center in camera frame.
    ClassVoxel* class_voxel = nullptr;

    if (class_block) {
      class_voxel = &class_block->getVoxelByLinearIndex(i);
    }
    if (submap->hasClassLayer() &&
        (!config_.update_only_tracked_submaps || submap->wasTracked())) {
      CHECK_NOTNULL(class_block);
      CHECK_NOTNULL(class_voxel);
    }

    if (updateVoxel(interpolator, &voxel, p_C, input, submap_id,
                    is_free_space_submap, truncation_distance, voxel_size,
                    class_voxel)) {
      was_updated = true;
    }
  }
  if (was_updated) {
    block.setUpdatedAll();
  }
}

bool MultiClassProjectiveIntegrator::updateVoxel(
    InterpolatorBase* interpolator, TsdfVoxel* voxel, const Point& p_C,
    const InputData& input, const int submap_id,
    const bool is_free_space_submap, const float truncation_distance,
    const float voxel_size, ClassVoxel* class_voxel) const {
  // Compute the signed distance. This also sets up the interpolator.
  float sdf;
  if (!computeSignedDistance(p_C, interpolator, &sdf)) {
    return false;
  }
  if (sdf < -truncation_distance) {
    return false;
  }

  // Compute the weight of the measurement.
  const float weight = computeWeight(p_C, voxel_size, truncation_distance, sdf);

  // Truncate the sdf to the truncation band.
  sdf = std::min(sdf, truncation_distance);

  // Only merge color and classification data near the surface.
  if (std::abs(sdf) >= truncation_distance || is_free_space_submap) {
    updateVoxelValues(voxel, sdf, weight);
  } else {
    const Color color = interpolator->interpolateColor(input.colorImage());
    updateVoxelValues(voxel, sdf, weight, &color);

    // Update the class voxel.
    if (class_voxel) {
      updateClassVoxel(interpolator, class_voxel, input, submap_id);
    }
  }
  return true;
}

void MultiClassProjectiveIntegrator::updateClassVoxel(
    InterpolatorBase* interpolator, ClassVoxel* voxel, const InputData& input,
    const int submap_id) const {
  const int id = interpolator->interpolateID(input.idImage());
  if (id_classes_set_1_.find(id) != id_classes_set_1_.end()) {
    // use classification for class set 1
    if (config_.use_instance_classification) {
      voxel->incrementCount(id);
    } else {
      // NOTE(schmluk): id_to_class should always exist since it's created based
      // on the input.
      voxel->incrementCount(submap_id_to_class_.at(id));
    }
  } else {
    // use classification for class set 2
    // use binary classification
    // Use ID 0 for belongs, 1 for does not belong.
    if (config_.use_instance_classification) {
      // Just count how often the assignments were right.
      if (instance_to_active_submap_ids_.count(id) != 0) {
        const int input_submap_id = instance_to_active_submap_ids_.at(id);
        voxel->incrementCount(1 -
                              static_cast<int>(input_submap_id == submap_id));
      } else {
        voxel->incrementCount(1);
      }
    } else {
      // Only the class needs to match.
      auto it = submap_id_to_class_.find(submap_id);
      auto it2 = submap_id_to_class_.find(
          interpolator->interpolateID(input.idImage()));
      if (it != submap_id_to_class_.end() && it2 != submap_id_to_class_.end()) {
        voxel->incrementCount(1 - static_cast<int>(it->second == it2->second));
      } else {
        voxel->incrementCount(1);
      }
    }
  }
}

void MultiClassProjectiveIntegrator::allocateNewBlocks(
    SubmapCollection* submaps, const InputData& input) {
  // This method also resets the depth image.
  range_image_.setZero();
  max_range_in_image_ = 0.f;
  // Parse through each point to allocate instance + background blocks.
  std::unordered_set<Submap*> touched_submaps;
  for (int v = 0; v < input.depthImage().rows; v++) {
    for (int u = 0; u < input.depthImage().cols; u++) {
      const cv::Vec3f& vertex = input.vertexMap().at<cv::Vec3f>(v, u);
      const Point p_C(vertex[0], vertex[1], vertex[2]);
      const float ray_distance = p_C.norm();
      range_image_(v, u) = ray_distance;
      if (ray_distance > cam_config_->max_range ||
          ray_distance < cam_config_->min_range) {
        continue;
      }
      max_range_in_image_ = std::max(max_range_in_image_, ray_distance);
      const int id = input.idImage().at<int>(v, u);
      bool is_on_set_1 = id_classes_set_1_.find(id) != id_classes_set_1_.end();
      bool submap_id_exists = submaps->submapWithClassIdExists(id);
      Submap* submap = nullptr;
      if (is_on_set_1) {
        submap = submaps->getBackground();
      } else if (submap_id_exists) {
        const auto& all_active_submaps_with_id =
            submaps->getActiveSubmapWithClassId(id);
        if (!all_active_submaps_with_id.empty()) {
          submap = all_active_submaps_with_id[0];
        } else {
          submap_id_exists = false;
        }
      }
      if (is_on_set_1 || submap_id_exists) {
        const int voxels_per_side = submap->getConfig().voxels_per_side;
        const Point p_S = submap->getT_S_M() * input.T_M_C() * p_C;
        const voxblox::BlockIndex block_index =
            submap->getTsdfLayer().computeBlockIndexFromCoordinates(p_S);
        const auto block =
            submap->getTsdfLayerPtr()->allocateBlockPtrByIndex(block_index);
        if (submap->hasClassLayer()) {
          submap->getClassLayerPtr()->allocateBlockPtrByIndex(block_index);
        }
        touched_submaps.insert(submap);
      }
    }
  }
  max_range_in_image_ = std::min(max_range_in_image_, cam_config_->max_range);

  // Allocate all potential free space blocks.
  if (submaps->submapIdExists(submaps->getActiveFreeSpaceSubmapID())) {
    Submap* space =
        submaps->getSubmapPtr(submaps->getActiveFreeSpaceSubmapID());
    const float block_size = space->getTsdfLayer().block_size();
    const float block_diag_half = std::sqrt(3.f) * block_size / 2.f;
    const Transformation T_C_S = input.T_M_C().inverse() * space->getT_M_S();
    const Point camera_S = T_C_S.inverse().getPosition();  // T_S_C
    const int max_steps = std::floor((max_range_in_image_ + block_diag_half) /
                                     space->getTsdfLayer().block_size());
    for (int x = -max_steps; x <= max_steps; ++x) {
      for (int y = -max_steps; y <= max_steps; ++y) {
        for (int z = -max_steps; z <= max_steps; ++z) {
          const Point offset(x, y, z);
          const Point candidate_S = camera_S + offset * block_size;
          if (globals_->camera()->pointIsInViewFrustum(T_C_S * candidate_S,
                                                       block_diag_half)) {
            space->getTsdfLayerPtr()->allocateBlockPtrByCoordinates(
                candidate_S);
          }
        }
      }
    }
    space->getBoundingVolumePtr()->update();
  }

  // Update all bounding volumes. This is currently done in every integration
  // step since it's not too expensive and won't do anything if no new block
  // was allocated.
  for (auto& submap : touched_submaps) {
    submap->updateBoundingVolume();
  }
}

}  // namespace panoptic_mapping
