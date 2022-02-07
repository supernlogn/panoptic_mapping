#include "panoptic_mapping/map/pose_manager.h"

#include <algorithm>
#include <set>
#include <vector>

#define INVALID_POSE_ID_NUM -1

namespace panoptic_mapping {

void PoseManager::copy(PoseManager* other) const {
  // a very time-expensive copy operation
  // because every copy is a deep-copy
  other->poses_info_ = poses_info_;
  other->submap_id_to_pose_id_ = submap_id_to_pose_id_;
  other->pose_id_to_submap_id_ = pose_id_to_submap_id_;
  other->next_pose_id_index_ = next_pose_id_index_;
}

void PoseManager::correctPoseRangeTransformation(const poseIdType start_pose_id,
                                                 const poseIdType end_pose_id,
                                                 const Transformation& T_corr) {
  LOG_IF(INFO, true) << "correcting " << (end_pose_id - start_pose_id + 1)
                     << " transformations with " << T_corr << std::endl;
  for (auto pose_id = 0; pose_id <= end_pose_id; ++pose_id) {
    assert(poses_info_.find(pose_id) != poses_info_.end());
    const poseIdxType pose_idx = poses_info_[pose_id].pose_idx;
    Transformation& pose_at_pose_idx = poses_info_[pose_idx].pose;
    const Transformation& init_pose_at_pose_idx =
        poses_info_[pose_idx].pose_init;
    pose_at_pose_idx = init_pose_at_pose_idx * T_corr;
  }
}

void PoseManager::correctPoseRangeTransformation(
    const std::set<PoseManager::poseIdType>& pose_ids,
    const Transformation& T_corr) {
  LOG_IF(INFO, true) << "correcting " << pose_ids.size()
                     << " transformations with " << T_corr << std::endl;
  for (const auto pose_id : pose_ids) {
    assert(poses_info_.find(pose_id) != poses_info_.end());
    const poseIdxType pose_idx = poses_info_[pose_id].pose_idx;
    Transformation& pose_at_pose_idx = poses_info_[pose_idx].pose;
    const Transformation& init_pose_at_pose_idx =
        poses_info_[pose_idx].pose_init;
    pose_at_pose_idx = init_pose_at_pose_idx * T_corr;
  }
}

void PoseManager::correctPoseRangeTransformation(
    const std::vector<PoseManager::poseIdType>& pose_ids,
    const Transformation& T_corr) {
  LOG_IF(INFO, true) << "correcting " << pose_ids.size()
                     << " transformations with " << T_corr << std::endl;
  for (const auto pose_id : pose_ids) {
    assert(poses_info_.find(pose_id) != poses_info_.end());
    const poseIdxType pose_idx = poses_info_[pose_id].pose_idx;
    Transformation& pose_at_pose_idx = poses_info_[pose_idx].pose;
    const Transformation& init_pose_at_pose_idx =
        poses_info_[pose_idx].pose_init;
    pose_at_pose_idx = init_pose_at_pose_idx * T_corr;
  }
}

void PoseManager::updateSinglePoseTransformation(
    const poseIdType pose_id, const Transformation& new_pose) {
  const auto it = poses_info_.find(pose_id);
  if (it == poses_info_.end()) {
    // not found
    return;
  } else {
    const poseIdxType pose_idx = poses_info_[pose_id].pose_idx;
    poses_info_[pose_idx].pose = new_pose;
  }
}

PoseManager::poseIdType PoseManager::createPose(const Transformation& new_pose,
                                                const ros::Time& pose_time) {
  const poseIdType new_p_id = createNewPoseId();
  poses_info_.emplace(new_p_id, (PoseInformation){.time = pose_time,
                                                  .pose_idx = new_p_id,
                                                  .pose_init = new_pose,
                                                  .pose = new_pose});
  return new_p_id;
}

void PoseManager::addSubmapIdToPose(const poseIdType pose_id,
                                    const submapIdType submap_id) {
  assert(poses_info_.find(pose_id) != poses_info_.end());
  submap_id_to_pose_id_[submap_id].insert(pose_id);
  pose_id_to_submap_id_[pose_id].insert(submap_id);
}

void PoseManager::addSubmapIdToPoses(const poseIdType submap_id,
                                     const std::set<poseIdType>& pose_ids) {
  assert(submap_id_to_pose_id_.find(submap_id) != submap_id_to_pose_id_.end());
  for (const PoseManager::poseIdType p_id : pose_ids) {
    addSubmapIdToPose(p_id, submap_id);
  }
}

void PoseManager::removeSubmapIdFromPose(const poseIdType pose_id,
                                         const submapIdType submap_id) {
  assert(poses_info_.find(pose_id) != poses_info_.end());
  submap_id_to_pose_id_[submap_id].erase(pose_id);
  pose_id_to_submap_id_[pose_id].erase(submap_id);
}

void PoseManager::removeSubmapIdFromPoses(
    const PoseManager::submapIdType submap_id,
    const std::set<poseIdType>& pose_ids) {
  for (const auto p_id : pose_ids) {
    removeSubmapIdFromPose(p_id, submap_id);
  }
}

void PoseManager::removeSubmapIdFromAllPoses(
    const PoseManager::submapIdType submap_id) {
  const std::set<poseIdType>& submaps_poses = submap_id_to_pose_id_[submap_id];
  removeSubmapIdFromPoses(submap_id, submaps_poses);
  submap_id_to_pose_id_.erase(submap_id);
}

void PoseManager::clear() {
  submap_id_to_pose_id_.clear();
  pose_id_to_submap_id_.clear();
  poses_info_.clear();
  next_pose_id_index_ = 0;  // reset next pose id index
}

Transformation PoseManager::getPoseCorrectionTF(
    const poseIdType pose_id, const Transformation& T_voxgraph,
    const Transformation& T_C_R) const {
  assert(poses_info_.find(pose_id) != poses_info_.end());
  const Transformation& pose_at_pose_id =
      gravityAlignPose(poses_info_.at(pose_id).pose_init * T_C_R);
  Transformation result = pose_at_pose_id.inverse() * T_voxgraph;
  assert(pose_at_pose_id * result == T_voxgraph);
  return result;
}

Transformation PoseManager::getPoseCorrectionTFInv(
    const poseIdType pose_id, const Transformation& T_voxgraph,
    const Transformation& T_C_R) const {
  assert(poses_info_.find(pose_id) != poses_info_.end());
  const Transformation& pose_at_pose_id =
      gravityAlignPose(poses_info_.at(pose_id).pose_init * T_C_R);
  Transformation result = T_voxgraph.inverse() * pose_at_pose_id;

  assert(result * T_voxgraph == pose_at_pose_id);
  return result;
}

std::set<PoseManager::submapIdType> PoseManager::getConnectedSubmaps(
    const PoseManager::submapIdType submap_id) const {
  std::set<submapIdType> ret;
  // TODO(supernlogn): make it faster
  for (const poseIdType p_id : submap_id_to_pose_id_.at(submap_id)) {
    for (const submapIdType s_id : pose_id_to_submap_id_.at(p_id)) {
      ret.insert(s_id);
    }
  }
  if (ret.find(submap_id) != ret.end()) {
    ret.erase(submap_id);
  }
  return ret;
}

PoseManager::PoseInformation PoseManager::getPoseInformation(
    const poseIdType pose_id) const {
  const auto it = poses_info_.find(pose_id);
  assert(it != poses_info_.end());
  return it->second;
}

Transformation PoseManager::getPoseTransformation(
    const poseIdType pose_id) const {
  assert(poses_info_.find(pose_id) != poses_info_.end());
  const Transformation& pose_transformation = poses_info_.at(pose_id).pose;
  return pose_transformation;
}

Transformation PoseManager::getInitPoseTransformation(
    const poseIdType pose_id) const {
  assert(poses_info_.find(pose_id) != poses_info_.end());
  const Transformation& init_pose_transformation =
      poses_info_.at(pose_id).pose_init;
  return init_pose_transformation;
}

PoseManager::poseIdType PoseManager::getPoseIdAtTime(
    const ros::Time time) const {
  // This parses the entire container with
  // PoseInformation sequentially trying to find
  // the pose id at the given time.
  // If no PoseInformation exists for the specified time exists,
  // an INVALID_POSE_ID_NUM is returned
  auto it_result = poses_info_.end();
  for (auto it = poses_info_.begin(); it != poses_info_.end(); ++it) {
    if (it->second.time == time) {
      it_result = it;
      break;
    }
  }
  assert(it_result != poses_info_.end());
  if (it_result == poses_info_.end()) {
    return INVALID_POSE_ID_NUM;
  }
  return it_result->first;
}

const PoseManager::PoseInformation* PoseManager::getPoseInformationAtTime(
    const ros::Time time) const {
  // This parses the entire container with PoseInformation sequentially trying
  // to find a pointer to the PoseInformation at the given time.
  // If no PoseInformation for the specified time exists, a nullptr is returned
  auto it_result = poses_info_.end();
  for (auto it = poses_info_.begin(); it != poses_info_.end(); ++it) {
    if (it->second.time == time) {
      it_result = it;
      break;
    }
  }
  assert(it_result != poses_info_.end());
  if (it_result == poses_info_.end()) {
    return nullptr;
  }
  const PoseManager::PoseInformation* result = &(it_result->second);
  return result;
}

Transformation PoseManager::getPoseTransformationAtTime(
    const ros::Time time) const {
  const PoseManager::PoseInformation* p_info = getPoseInformationAtTime(time);
  return p_info->pose;
}

Transformation PoseManager::gravityAlignPose(
    const Transformation& input_pose) const {
  // Use the logarithmic map to get the pose's [x, y, z, r, p, y] components
  Transformation::Vector6 T_vec = input_pose.log();

  // Print a warning if the original pitch & roll components were large
  constexpr float angle_threshold_rad = 30.f /* deg */ / 180.f * M_PI;
  // Set the roll and pitch to zero
  T_vec[3] = 0;
  T_vec[4] = 0;

  // Return the gravity aligned pose as a translation + quaternion,
  // using the exponential map
  return Transformation::exp(T_vec);
}

bool PoseManager::hasPose(const poseIdType pose_id) const {
  auto it = poses_info_.find(pose_id);
  bool has_pose = (it != poses_info_.end());
  return has_pose;
}

};  // namespace panoptic_mapping
