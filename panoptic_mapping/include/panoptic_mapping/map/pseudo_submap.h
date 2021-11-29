#ifndef PANOPTIC_MAPPING_MAP_PSEUDO_SUBMAP_H_
#define PANOPTIC_MAPPING_MAP_PSEUDO_SUBMAP_H_

#include "panoptic_mapping/map/submap.h"
#include "panoptic_mapping/map/pseudo_submap_bounding_volume.h"

namespace panoptic_mapping {

class PseudoSubmap {
 // This class is like submap, but is not managed by any MapManagers
 // or instance managers. Its purpose is to keep and copy submap data
 // without triggering any managers.
 public:
  // Construction.
  PseudoSubmap() {};
  PseudoSubmap(const Submap & submap);

  virtual ~PseudoSubmap() = default;
  // copy constructor
  void clone(PseudoSubmap & other) const;
  // Const accessors.
  int getClassID() const { return class_id_; }
  PanopticLabel getLabel() const { return label_; }
  const std::string& getName() const { return name_; }
  const std::string& getFrameName() const { return frame_name_; }
  const TsdfLayer& getTsdfLayer() const { return *tsdf_layer_; }
  const ClassLayer& getClassLayer() const { return *class_layer_; }
  const voxblox::MeshLayer& getMeshLayer() const { return *mesh_layer_; }
  const Transformation& getT_M_S() const { return T_M_S_; }
  const Transformation& getT_S_M() const { return T_M_S_inv_; }
  bool hasClassLayer() const { return has_class_layer_; }
  const std::vector<IsoSurfacePoint>& getIsoSurfacePoints() const {
    return iso_surface_points_;
  }
  ChangeState getChangeState() const { return change_state_; }

  // Modifying accessors
  std::shared_ptr<TsdfLayer>& getTsdfLayerPtr() { return tsdf_layer_; }
  std::shared_ptr<ClassLayer>& getClassLayerPtr() { return class_layer_; }
  std::shared_ptr<voxblox::MeshLayer>& getMeshLayerPtr() { return mesh_layer_; }
  Submap::PoseIdHistory& getPoseHistory() { return pose_id_history_; }
  const PseudoSubmapBoundingVolume& getBoundingVolume() const {
    return bounding_volume_;
  }

  /**
   * @brief Update all dynamically computable quantities.
   *
   * @param only_updated_blocks If false, recompute all quantities from scratch.
   * If true, recompute based on what is flagged updated.
   */
  void updateEverything(bool only_updated_blocks = true);

  /**
   * @brief Update the bounding volume based on all allocated blocks.
   */
  void updateBoundingVolume();

  /**
   * @brief Update the mesh based on the current tsdf blocks. Set
   * only_updated_blocks true for incremental mesh updates, false for a full
   * re-computation.
   *
   * @param only_updated_blocks If false, recompute the mesh from scratch. If
   * true, update based on the updated(kMesh) flag of the TSDF layer.
   * @param use_class_layer Set to true to use the class layer if it is
   * available.
   */
  void updateMesh(bool only_updated_blocks = true, bool use_class_layer = true);

  /**
   * @brief Compute the iso-surface points of the submap based on its current
   * mesh. Currently all surface points are computed from scratch every time,
   * but since they are currently only computed when a submap is finished it
   * should be fine. This function utilizes the stored mesh so make sure
   * updateMesh is called earlier.
   */
  void computeIsoSurfacePoints();

  /**
   * @brief Removes non-belonging points from the TSDF and deletes the class
   * layer. Uses the provided manipulator to perform the class layer
   * integration.
   *
   * @param manipulator Manipulator used to carry out the application
   * of the class layer.
   * @param clear_class_layer True: erase the class layer. False: keep the class
   * layer for lookups, but no further manipulations.
   * @return True if any blocks remain, false if the TSDF map was cleared.
   */
  bool applyClassLayer(const LayerManipulator& manipulator,
                       bool clear_class_layer = true);

 protected:
  int class_id_ = -1;
  PanopticLabel label_ = PanopticLabel::kUnknown;
  std::string name_ = "Unknown";

  // State.
  bool has_class_layer_ = false;
  ChangeState change_state_ = ChangeState::kNew;

  // Transformations.
  std::string frame_name_;
  Transformation T_M_S_;  // Transformation mission to submap.
  Transformation T_M_S_inv_;

  // Map.
  std::shared_ptr<TsdfLayer> tsdf_layer_;
  std::shared_ptr<ClassLayer> class_layer_;
  std::shared_ptr<voxblox::MeshLayer> mesh_layer_;
  std::vector<IsoSurfacePoint> iso_surface_points_;
  PseudoSubmapBoundingVolume bounding_volume_;

  // Processing.
  std::unique_ptr<MeshIntegrator> mesh_integrator_;

  // History of how the robot moved through the submap
  Submap::PoseIdHistory pose_id_history_;

 private:
  float truncation_distance_;
};

}

#endif