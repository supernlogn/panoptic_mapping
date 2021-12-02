#ifndef PANOPTIC_MAPPING_MAP_PSEUDO_SUBMAP_H_
#define PANOPTIC_MAPPING_MAP_PSEUDO_SUBMAP_H_

#include <string>
#include <memory>

#include "panoptic_mapping/map/submap.h"

namespace panoptic_mapping {

class PseudoSubmap {
// This class is like submap, but is not managed by any MapManagers
// or instance managers. Its purpose is to keep and copy submap data
// without triggering any managers.
 public:
  // Construction.
  PseudoSubmap() {}
  explicit PseudoSubmap(const Submap & submap);

  virtual ~PseudoSubmap() = default;
  // copy constructor
  void clone(PseudoSubmap * other) const;
  // Const accessors.
  int getClassID() const { return class_id_; }
  PanopticLabel getLabel() const { return label_; }
  const std::string& getName() const { return name_; }
  const std::string& getFrameName() const { return frame_name_; }
  const TsdfLayer& getTsdfLayer() const { return *tsdf_layer_; }
  const Transformation& getT_M_S() const { return T_M_S_; }
  const Transformation& getT_S_M() const { return T_M_S_inv_; }
  ChangeState getChangeState() const { return change_state_; }
  const Submap::PoseIdHistory& getPoseHistoryConst() const {
                                      return pose_id_history_; }
  // Modifying accessors
  std::shared_ptr<TsdfLayer>& getTsdfLayerPtr() { return tsdf_layer_; }
  Submap::PoseIdHistory& getPoseHistory() { return pose_id_history_; }

 protected:
  int class_id_ = -1;
  PanopticLabel label_ = PanopticLabel::kUnknown;
  std::string name_ = "Unknown";

  // State.
  ChangeState change_state_ = ChangeState::kNew;

  // Transformations.
  std::string frame_name_;
  Transformation T_M_S_;  // Transformation mission to submap.
  Transformation T_M_S_inv_;

  // Map.
  std::shared_ptr<TsdfLayer> tsdf_layer_;

  // History of how the robot moved through the submap
  Submap::PoseIdHistory pose_id_history_;

 private:
  float truncation_distance_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_PSEUDO_SUBMAP_H_
