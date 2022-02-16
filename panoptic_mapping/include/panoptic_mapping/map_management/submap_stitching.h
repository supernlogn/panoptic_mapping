#ifndef PANOPTIC_MAPPING_MAP_MANAGEMENT_SUBMAP_STITCHING_H_
#define PANOPTIC_MAPPING_MAP_MANAGEMENT_SUBMAP_STITCHING_H_

#include <map>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/common/plane_type.h"
#include "panoptic_mapping/map/submap.h"

#include "voxgraph/frontend/submap_collection/bounding_box.h"

namespace panoptic_mapping {
// TODO(supernlogn): see if this can occupy less memory
typedef struct PointIndexType {
  int block_index;
  int linear_index;
};

class SubmapStitching {
 public:
  typedef int ClassID;
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;
    double z_threshold = 0.15;
    double xy_threshold = 0.01;
    int max_walls = 4;
    int max_floors = 4;
    int max_ceilings = 4;
    double normal_cluster_threshold = 0.1;     // no-unit
    double position_cluster_threshold = 0.05;  // [m]
    float max_outlier_percentage = 0.10;
    Config() { setConfigName("SubmapStitching"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };
  explicit SubmapStitching(const Config& config);
  virtual ~SubmapStitching() = default;

  void processSubmap(const Submap& s);
  void findSubmapPlanes(const Submap& s);

  bool stitch(const Submap& SubmapA, Submap* submapB);
  int findNeighboors(const Submap& s, std::vector<int>* neighboor_ids);
  /**
   * @brief Get the Max Num Planes Per class id
   *
   * @param class_id the class of the object
   * @return int maximum number of planes
   */
  int getMaxNumPlanesPerType(ClassID class_id) const;

 private:
  // for individual submap
  void findSubmapPlanes(const voxblox::MeshLayer& meshLayer);
  bool planeRansac(
      std::map<ClassID, std::vector<Eigen::Hyperplane<float, 3> > >*
          merged_result,
      const int num_iterations);
  std::vector<Eigen::Hyperplane<float, 3> > ransacSample(
      const std::vector<PointIndexType>& p_indices, const ClassID class_id);
  std::map<ClassID, std::vector<PointIndexType> > applyClassPreFilter();
  Eigen::Hyperplane<float, 3> createPlaneFrom3Points(const Point& p1,
                                                     const Point& p2,
                                                     const Point& p3) const;
  // return the number of outlier points given a set of planes and the whole
  // point set.
  std::pair<int, BoundingBoxType> ransac_check(
      const std::vector<Eigen::Hyperplane<float, 3> >& hyperplanes,
      const std::vector<PointIndexType>& p_indices);

  // for stitching submap
  void matchNeighboorPlanes(const Submap& SubmapA, const Submap& SubmapB,
                            const std::vector<int>& neighboor_ids);

  const Config config_;
  std::map<int, int> submap_neighboors;
  std::map<PlaneType::PlaneID, PlaneType> clustered_submap_planes_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_MANAGEMENT_SUBMAP_STITCHING_H_
