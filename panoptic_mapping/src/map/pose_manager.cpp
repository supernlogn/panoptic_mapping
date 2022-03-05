#include "panoptic_mapping/map/pose_manager.h"

#include <algorithm>
#include <set>
#include <string>
#include <vector>

#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <voxgraph/tools/io.h>

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

void PoseManager::correctPoseInfo(PoseManager::PoseInformation* pose_info,
                                  const Transformation& T_corr) {
  pose_info->pose = pose_info->pose_init * T_corr;
}

void PoseManager::correctPoseRangeTransformation(const poseIdType start_pose_id,
                                                 const poseIdType end_pose_id,
                                                 const Transformation& T_corr) {
  for (auto pose_id = 0; pose_id <= end_pose_id; ++pose_id) {
    CHECK(poses_info_.find(pose_id) != poses_info_.end());
    correctPoseInfo(&poses_info_[pose_id], T_corr);
  }
}

void PoseManager::correctPoseRangeTransformation(
    const std::set<PoseManager::poseIdType>& pose_ids,
    const Transformation& T_corr) {
  for (const auto pose_id : pose_ids) {
    CHECK(poses_info_.find(pose_id) != poses_info_.end());
    correctPoseInfo(&poses_info_[pose_id], T_corr);
  }
}

void PoseManager::correctPoseRangeTransformation(
    const std::vector<PoseManager::poseIdType>& pose_ids,
    const Transformation& T_corr) {
  for (const auto pose_id : pose_ids) {
    CHECK(poses_info_.find(pose_id) != poses_info_.end());
    correctPoseInfo(&poses_info_[pose_id], T_corr);
  }
}

void PoseManager::correctSubmapTrajectory(
    const PoseManager::submapIdType submap_id, const Transformation& T_corr) {
  const auto& submap_pose_ids = submap_id_to_pose_id_.at(submap_id);
  correctPoseRangeTransformation(submap_pose_ids, T_corr);
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
  CHECK(poses_info_.find(pose_id) != poses_info_.end());
  submap_id_to_pose_id_[submap_id].insert(pose_id);
  pose_id_to_submap_id_[pose_id].insert(submap_id);
}

void PoseManager::addSubmapIdToPoses(const poseIdType submap_id,
                                     const std::set<poseIdType>& pose_ids) {
  CHECK(submap_id_to_pose_id_.find(submap_id) != submap_id_to_pose_id_.end());
  for (const PoseManager::poseIdType p_id : pose_ids) {
    addSubmapIdToPose(p_id, submap_id);
  }
}

void PoseManager::removeSubmapIdFromPose(const poseIdType pose_id,
                                         const submapIdType submap_id) {
  CHECK(poses_info_.find(pose_id) != poses_info_.end());
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

bool PoseManager::saveAllPosesToFile(const std::string& file_path) const {
  std::vector<geometry_msgs::PoseStamped> trajectoryPoses;
  for (const auto& p_info_pair : poses_info_) {
    const auto& p_info = p_info_pair.second;
    const geometry_msgs::PoseStamped pose_stamped_msg = getPoseMessage(p_info);
    trajectoryPoses.push_back(pose_stamped_msg);
  }
  bool res = voxgraph::io::savePoseHistoryToFile(file_path, trajectoryPoses);
  return res;
}

bool PoseManager::savePoseIdsToFile(const std::string& file_path,
                                    const std::vector<int>& pose_ids) const {
  std::vector<geometry_msgs::PoseStamped> trajectoryPoses;
  for (const int id : pose_ids) {
    trajectoryPoses.push_back(getPoseMessage(id));
  }
  bool res = voxgraph::io::savePoseHistoryToFile(file_path, trajectoryPoses);
  return res;
}

geometry_msgs::PoseStamped PoseManager::getPoseMessage(
    const int pose_id) const {
  const auto p_info = getPoseInformation(pose_id);
  return getPoseMessage(p_info);
}

geometry_msgs::PoseStamped PoseManager::getPoseMessage(
    const PoseManager::PoseInformation& p_info) const {
  geometry_msgs::PoseStamped pose_stamped_msg;
  // avoid time 0
  if (p_info.time < ros::TIME_MIN) {
    pose_stamped_msg.header.stamp = ros::TIME_MIN;
  } else {
    pose_stamped_msg.header.stamp = p_info.time;
  }
  pose_stamped_msg.header.frame_id = "world";
  tf::poseKindrToMsg(p_info.pose.cast<double>(), &pose_stamped_msg.pose);
  return pose_stamped_msg;
}

Transformation PoseManager::getPoseCorrectionTF(
    const poseIdType pose_id, const Transformation& T_M_R_voxgraph,
    const Transformation& T_C_R) const {
  CHECK(poses_info_.find(pose_id) != poses_info_.end());
  const Transformation& T_M_R_voxgraph_init =
      gravityAlignPose(poses_info_.at(pose_id).pose_init * T_C_R);
  Transformation result =
      T_C_R * T_M_R_voxgraph_init.inverse() * T_M_R_voxgraph * T_C_R.inverse();
  // CHECK(T_M_R_voxgraph_init * result == T_M_R_voxgraph);
  return result;
}

Transformation PoseManager::getPoseCorrectionTFInv(
    const poseIdType pose_id, const Transformation& T_M_R_voxgraph,
    const Transformation& T_C_R) const {
  CHECK(poses_info_.find(pose_id) != poses_info_.end());
  const Transformation& T_M_R_voxgraph_init =
      gravityAlignPose(poses_info_.at(pose_id).pose_init * T_C_R);
  Transformation result = T_M_R_voxgraph.inverse() * T_M_R_voxgraph_init;

  CHECK(result * T_M_R_voxgraph == T_M_R_voxgraph_init);
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
  CHECK(it != poses_info_.end());
  return it->second;
}

Transformation PoseManager::getPoseTransformation(
    const poseIdType pose_id) const {
  CHECK(poses_info_.find(pose_id) != poses_info_.end());
  const Transformation& pose_transformation = poses_info_.at(pose_id).pose;
  return pose_transformation;
}

Transformation PoseManager::getInitPoseTransformation(
    const poseIdType pose_id) const {
  CHECK(poses_info_.find(pose_id) != poses_info_.end());
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
  CHECK(it_result != poses_info_.end());
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
  CHECK(it_result != poses_info_.end());
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
  // Set the roll and pitch to zero
  T_vec[3] = 0;
  T_vec[4] = 0;

  // Return the gravity aligned pose as a translation + quaternion,
  // using the exponential map
  return Transformation::exp(T_vec);
}

ros::Time PoseManager::getSubmapStartTime(const submapIdType submap_id) const {
  const auto& submap_poses = submap_id_to_pose_id_.at(submap_id);
  int start_pose_id = *submap_poses.begin();
  const PoseInformation& p_info_start = poses_info_.at(start_pose_id);
  return p_info_start.time;
}

ros::Time PoseManager::getSubmapEndTime(const submapIdType submap_id) const {
  const auto& submap_poses = submap_id_to_pose_id_.at(submap_id);
  int end_pose_id = *submap_poses.rbegin();
  const PoseInformation& p_info_end = poses_info_.at(end_pose_id);
  return p_info_end.time;
}

bool PoseManager::hasPose(const poseIdType pose_id) const {
  auto it = poses_info_.find(pose_id);
  bool has_pose = (it != poses_info_.end());
  return has_pose;
}

};  // namespace panoptic_mapping
