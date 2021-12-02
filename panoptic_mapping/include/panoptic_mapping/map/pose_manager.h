#ifndef PANOPTIC_MAPPING_MAP_MANAGEMENT_POSE_MANAGER_H_
#define PANOPTIC_MAPPING_MAP_MANAGEMENT_POSE_MANAGER_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <map>
#include <set>
#include <iostream>

#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "tf/tf.h"

namespace panoptic_mapping {

class PoseManager {
 public:
  // const static int invalidIdNum;
  typedef int poseIdType;
  typedef int poseIdxType;
  typedef int submapIdType;

  typedef struct { 
      ros::Time time;
      poseIdxType pose_idx;
      Transformation pose;
  } PoseInformation;

  PoseManager() = default;
  virtual ~PoseManager() = default;

  // Access to the global id manager via singleton.
  static PoseManager* getGlobalInstance() {
    static PoseManager instance;
    return &instance;
  }
  // copying information
  void copy(PoseManager & other) const;
  // updating information
  void correctPoseRangeTransformation(
                        const poseIdType start_pose_id,
                        const poseIdType end_pose_id,
                        const Transformation & corr_transformation);

  void correctPoseRangeTransformation(
                        const std::set<poseIdType> & pose_ids,
                        const Transformation & corr_transformation);

  void correctPoseRangeTransformation(
                        const std::vector<poseIdType> & pose_ids,
                        const Transformation & corr_transformation);

  void updateSinglePoseTransformation(const poseIdType pose_id,
                                      const Transformation & new_pose);

  // adding information
  poseIdType createPose(const Transformation & new_pose,
                        const ros::Time & pose_time);

  void addSubmapIdToPose(const poseIdType pose_id,
                         const submapIdType submap_id);

  void addSubmapIdToPoses(const poseIdType submap_id,
                          const std::set<poseIdType> & pose_ids);

  // removing information
  void removeSubmapIdFromPoses(const submapIdType submap_id,
                          const std::set<poseIdType> & pose_ids);

  void removeSubmapIdFromAllPoses(const submapIdType submap_id);

  /**
   * @brief clears data contained in private member containers
   */
  void clear();

  /** 
   * @brief erases submap id from submaps_id_associated of
   * each pose information
   * The poses that will be affected are those with id inside pose_ids 
   **/
  void removeSubmapIdFromPose(const poseIdType pose_id, 
                              const submapIdType submap_id);

  // const accessors -- computing transformations
  /** @brief return a way to turn the pose at pose_id
  * to other_pose by multiplying the result
  * so that the below sentence holds true:
  * getPoseCorrectionTF(pose_id, other_pose) * pose@pose_id == other_pose
  **/
  Transformation getPoseCorrectionTF(const poseIdType pose_id, 
                               const Transformation & other_pose) const;

  /**
   * @brief return a way to turn the other_pose
   * to the pose at pose_id by multiplying the result 
   * so that the below sentence holds true:
   * getPoseCorrectionTF(pose_id, other_pose) * other_pose == pose@pose_id
   **/
  Transformation getPoseCorrectionTFInv(const poseIdType pose_id, 
                              const Transformation & other_pose) const;

  // const accessors -- fetching elements
  PoseInformation getPoseInformation(const poseIdType pose_id) const;
  Transformation getPoseTransformation(const poseIdType pose_id) const;
  poseIdType getPoseIdAtTime(const ros::Time time) const;
  const PoseInformation * getPoseInformationAtTime(const ros::Time time) const;
  Transformation getPoseTransformationAtTime(const ros::Time time) const;
  bool hasPose(const poseIdType pose_id) const;

 private:
  std::map<poseIdType, PoseInformation> poses_info_;
  std::map<submapIdType, std::set<poseIdType>> submap_id_to_pose_id_;
  std::map<poseIdType, std::set<submapIdType>> pose_id_to_submap_id_;
  poseIdType next_pose_id_index_ = 0;

  // handling next_pose access/creation
  poseIdType createNewPoseId() {
    const poseIdxType ret = next_pose_id_index_;
    ++next_pose_id_index_;
    return ret;
  }
};

}; // panoptic_mapping

#endif // PANOPTIC_MAPPING_MAP_MANAGEMENT_POSE_MANAGER_H_