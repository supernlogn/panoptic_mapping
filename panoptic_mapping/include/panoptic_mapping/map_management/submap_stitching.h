#ifndef PANOPTIC_MAPPING_MAP_MANAGEMENT_SUBMAP_STITCHING_H_
#define PANOPTIC_MAPPING_MAP_MANAGEMENT_SUBMAP_STITCHING_H_

#include <map>
#include <vector>

#include <Eigen/Geometry>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/map/submap.h"

namespace panoptic_mapping {

class SubmapStitching {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;
    double z_threshold = 0.15;
    double xy_threshold = 0.01;
    int max_walls = 4;
    int max_floors = 4;
    int max_ceilings = 4;
    double normal_cluster_threshold = 0.1;     // no-unit
    double position_cluster_threshold = 0.05;  // [m]
    Config() { setConfigName("SubmapStitching"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };
  struct PlaneType {
    typedef int PlaneID;
    Transformation plane_orientation;
    Eigen::Hyperplane<float, 3> plane;
    int class_id;
    size_t num_points;
    PlaneID plane_id;
    Eigen::Vector3f getPlaneNormal();
    double dist(const PlaneType&);
    Transformation::Vector6 operator-(const PlaneType&);
  };
  explicit SubmapStitching(const Config& config) : config_(config) {}
  virtual ~SubmapStitching() = default;

  void processSubmap(const Submap& s);
  void findSubmapPlanes(const Submap& s);

  bool stitch(const Submap& SubmapA, Submap* submapB);
  int findNeighboors(const Submap& s, std::vector<int>* neighboor_ids);

 private:
  // for individual submap
  void findMeshPlanes(const voxblox::MeshLayer& meshLayer);
  // for stitching submap
  void findNeighboorPlanes(const Submap& SubmapA, const Submap& SubmapB,
                           std::vector<int> neighboor_ids);

  const Config config_;
  std::map<int, int> submap_neighboors;
  std::map<PlaneType::PlaneID, PlaneType> clustered_submap_planes_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_MANAGEMENT_SUBMAP_STITCHING_H_
