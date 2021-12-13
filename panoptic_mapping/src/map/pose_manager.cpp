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
    pose_at_pose_idx = pose_at_pose_idx * T_corr;
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
    pose_at_pose_idx = pose_at_pose_idx * T_corr;
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
    pose_at_pose_idx = pose_at_pose_idx * T_corr;
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
  poses_info_.emplace(
      new_p_id, (PoseInformation){
                    .time = pose_time, .pose_idx = new_p_id, .pose = new_pose});
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
    const poseIdType pose_id, const Transformation& T_voxgraph) const {
  assert(poses_info_.find(pose_id) != poses_info_.end());
  const Transformation& pose_at_pose_id = poses_info_.at(pose_id).pose;
  Transformation result = pose_at_pose_id.inverse() * T_voxgraph;
  // LOG(INFO) << "getPoseCorrectionTF for Panoptic mapping pose: " << std::endl
  //           << pose_at_pose_id << std::endl
  //           << "and optimized pose:" << std::endl
  //           << T_voxgraph << std::endl
  //           << "correction transformation" << std::endl
  //           << result << std::endl;
  assert(pose_at_pose_id * result == T_voxgraph);
  return result;
}

Transformation PoseManager::getPoseCorrectionTFInv(
    const poseIdType pose_id, const Transformation& T_voxgraph) const {
  assert(poses_info_.find(pose_id) != poses_info_.end());
  const Transformation& pose_at_pose_id = poses_info_.at(pose_id).pose;
  Transformation result = T_voxgraph.inverse() * pose_at_pose_id;

  assert(result * T_voxgraph == pose_at_pose_id);
  return result;
}

void PoseManager::addPoseMemoriamForSubmap(const submapIdType submap_id,
                                           const Transformation& pose) {
  if (submap_id_to_memoriam_id_.find(submap_id) !=
      submap_id_to_memoriam_id_.end()) {
    // just push the pose to the memoriam vector
    const poseMemoriamIdType memoriam_id =
        submap_id_to_memoriam_id_.at(submap_id);
    memoriam_id_to_pose_vector_[memoriam_id].emplace_back(pose);
  } else {
    // create entry from submap_id to memoriam
    // create entry from memoriam to a new vector
    // add the pose to this vector
    const poseMemoriamIdType memoriam_id = memoriam_id_to_pose_vector_.size();
    submap_id_to_memoriam_id_[submap_id] = memoriam_id;
    memoriam_id_to_pose_vector_[memoriam_id] = std::vector<Transformation>();
    memoriam_id_to_pose_vector_[memoriam_id].emplace_back(pose);
  }
}

void PoseManager::addPoseMemoriamForPose(const poseIdType pose_id) {
  const Transformation& past_pose = getPoseTransformation(pose_id);
  if (pose_id_to_memoriam_id_.find(pose_id) != pose_id_to_memoriam_id_.end()) {
    // just push the pose to the memoriam vector
    const poseMemoriamIdType memoriam_id = pose_id_to_memoriam_id_.at(pose_id);
    memoriam_id_to_pose_vector_[memoriam_id].emplace_back(past_pose);
  } else {
    // create entry from pose_id to memoriam
    // create entry from memoriam to a new vector
    // add the pose to this vector
    const poseMemoriamIdType memoriam_id = memoriam_id_to_pose_vector_.size();
    pose_id_to_memoriam_id_[pose_id] = memoriam_id;
    memoriam_id_to_pose_vector_[memoriam_id] = std::vector<Transformation>();
    memoriam_id_to_pose_vector_[memoriam_id].emplace_back(past_pose);
  }
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

bool PoseManager::hasPose(const poseIdType pose_id) const {
  auto it = poses_info_.find(pose_id);
  bool has_pose = (it != poses_info_.end());
  return has_pose;
}

const std::vector<Transformation>& PoseManager::getPoseMemoriamForSubmap(
    const submapIdType submap_id) const {
  if (submap_id_to_memoriam_id_.find(submap_id) !=
      submap_id_to_memoriam_id_.end()) {
    const poseMemoriamIdType memoriam_id =
        submap_id_to_memoriam_id_.at(submap_id);
    const std::vector<Transformation>& ret =
        memoriam_id_to_pose_vector_.at(memoriam_id);
    return ret;
  }
  return invalid_vector_;
}

const std::vector<Transformation>& PoseManager::getPoseMemoriamForPose(
    const poseIdType pose_id) const {
  if (pose_id_to_memoriam_id_.find(pose_id) != pose_id_to_memoriam_id_.end()) {
    const poseMemoriamIdType memoriam_id = pose_id_to_memoriam_id_.at(pose_id);
    const std::vector<Transformation>& ret =
        memoriam_id_to_pose_vector_.at(memoriam_id);
    return ret;
  }
  return invalid_vector_;
}

const std::vector<Transformation>& PoseManager::getPoseMemoriamForMemoriam(
    const poseMemoriamIdType memoriam_id) const {
  if (memoriam_id_to_pose_vector_.find(memoriam_id) !=
      memoriam_id_to_pose_vector_.end()) {
    const std::vector<Transformation>& ret =
        memoriam_id_to_pose_vector_.at(memoriam_id);
    return ret;
  }
  return invalid_vector_;
}

};  // namespace panoptic_mapping
