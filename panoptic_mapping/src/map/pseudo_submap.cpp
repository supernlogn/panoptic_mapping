#include "panoptic_mapping/map/pseudo_submap.h"

#include <memory>
#include <sstream>
#include <vector>

#include <cblox/QuatTransformation.pb.h>
#include <cblox/utils/quat_transformation_protobuf_utils.h>
#include <voxblox/io/layer_io.h>

#include "panoptic_mapping/map_management/layer_manipulator.h"

namespace panoptic_mapping {

PseudoSubmap::PseudoSubmap(const Submap& submap):
  bounding_volume_(*this, submap.getBoundingVolume()) {
  has_class_layer_ = submap.hasClassLayer();
  change_state_ = submap.getChangeState();
  frame_name_ = submap.getFrameName();
  T_M_S_ = submap.getT_M_S();
  T_M_S_inv_ = submap.getT_S_M();
  truncation_distance_ = submap.getConfig().truncation_distance;
  // deep copies
  // by calling deep copy ctor
  tsdf_layer_ = std::make_shared<TsdfLayer>(submap.getTsdfLayer());
  class_layer_ = std::make_shared<ClassLayer>(submap.getClassLayer());
  mesh_layer_ = std::make_shared<MeshLayer>(submap.getMeshLayer());
  
  mesh_integrator_ = std::make_unique<MeshIntegrator>(
      submap.getConfig().mesh_config, tsdf_layer_, mesh_layer_, class_layer_,
      truncation_distance_);

  iso_surface_points_ = submap.getIsoSurfacePoints();
  pose_id_history_ = submap.getPoseHistory();
}

void PseudoSubmap::clone(PseudoSubmap & other) const {
  bounding_volume_.clone(other.bounding_volume_);
  other.has_class_layer_ = hasClassLayer();
  other.change_state_ = getChangeState();
  other.frame_name_ = getFrameName();
  other.T_M_S_ = getT_M_S();
  other.T_M_S_inv_ = getT_S_M();
  other.truncation_distance_ = truncation_distance_;
  // deep copies
  // by calling deep copy ctor
  other.tsdf_layer_ = std::make_shared<TsdfLayer>(getTsdfLayer());
  other.class_layer_ = std::make_shared<ClassLayer>(getClassLayer());
  other.mesh_layer_ = std::make_shared<MeshLayer>(getMeshLayer());
  
  other.iso_surface_points_ = getIsoSurfacePoints();
  other.pose_id_history_ = pose_id_history_;
}


void PseudoSubmap::updateEverything(bool only_updated_blocks) {
  updateBoundingVolume();
  updateMesh(only_updated_blocks);
  computeIsoSurfacePoints();
}

void PseudoSubmap::updateMesh(bool only_updated_blocks, bool use_class_layer) {
  // Use the default integrator config to have color always available.
  mesh_integrator_->generateMesh(only_updated_blocks, true,
                                 has_class_layer_ && use_class_layer);
}

void PseudoSubmap::computeIsoSurfacePoints() {
  iso_surface_points_ = std::vector<IsoSurfacePoint>();

  // Create an interpolator to interpolate the vertex weights from the TSDF.
  voxblox::Interpolator<TsdfVoxel> interpolator(tsdf_layer_.get());

  // Extract the vertices and verify.
  voxblox::BlockIndexList index_list;
  mesh_layer_->getAllAllocatedMeshes(&index_list);
  int ignored_points = 0;
  for (const voxblox::BlockIndex& index : index_list) {
    const Pointcloud& vertices = mesh_layer_->getMeshByIndex(index).vertices;
    iso_surface_points_.reserve(iso_surface_points_.size() + vertices.size());
    for (const Point& vertex : vertices) {
      // Try to interpolate the voxel weight and verify the distance.
      TsdfVoxel voxel;
      if (interpolator.getVoxel(vertex, &voxel, true)) {
        // if (voxel.distance > 0.1 * config_.voxel_size) {
        //   ignored_points++;
        // } else {
        iso_surface_points_.emplace_back(vertex, voxel.weight);
        // }
      }
    }
  }
  if (ignored_points > 0) {
    LOG(WARNING) << "Submap " << " (" << name_
                 << ") has " << ignored_points
                 << " iso-surface points with a distance > "
                 << 0.1 << "* voxel_size, these will be ignored.";
  }
}

void PseudoSubmap::updateBoundingVolume() { bounding_volume_.update(); }



bool PseudoSubmap::applyClassLayer(const LayerManipulator& manipulator,
                             bool clear_class_layer) {
  if (!has_class_layer_) {
    return true;
  }
  manipulator.applyClassificationLayer(tsdf_layer_.get(), *class_layer_,
                                       truncation_distance_);
  if (clear_class_layer) {
    class_layer_.reset();
    has_class_layer_ = false;
  }
  updateEverything();
  return tsdf_layer_->getNumberOfAllocatedBlocks() != 0;
}


} // namespace panoptic_mappin
