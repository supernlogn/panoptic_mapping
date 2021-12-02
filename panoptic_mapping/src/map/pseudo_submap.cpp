#include "panoptic_mapping/map/pseudo_submap.h"

#include <cblox/QuatTransformation.pb.h>
#include <cblox/utils/quat_transformation_protobuf_utils.h>
#include <voxblox/io/layer_io.h>

#include <memory>
#include <sstream>
#include <vector>

#include "panoptic_mapping/map_management/layer_manipulator.h"

namespace panoptic_mapping {

PseudoSubmap::PseudoSubmap(const Submap& submap) {
  frame_name_ = submap.getFrameName();
  T_M_S_ = submap.getT_M_S();
  T_M_S_inv_ = submap.getT_S_M();
  truncation_distance_ = submap.getConfig().truncation_distance;
  // deep copies
  // by calling deep copy ctor
  tsdf_layer_ = std::make_shared<TsdfLayer>(submap.getTsdfLayer());
  pose_id_history_ = submap.getPoseHistory();
}

void PseudoSubmap::clone(PseudoSubmap * other) const {
  other->change_state_ = getChangeState();
  other->frame_name_ = getFrameName();
  other->T_M_S_ = getT_M_S();
  other->T_M_S_inv_ = getT_S_M();
  other->truncation_distance_ = truncation_distance_;
  // deep copies
  // by calling deep copy ctor
  other->tsdf_layer_ = std::make_shared<TsdfLayer>(getTsdfLayer());
  other->pose_id_history_ = pose_id_history_;
}


}  // namespace panoptic_mapping
