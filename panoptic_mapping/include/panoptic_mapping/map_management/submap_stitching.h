#ifndef PANOPTIC_MAPPING_MAP_MANAGEMENT_SUBMAP_STITCHING_H_
#define PANOPTIC_MAPPING_MAP_MANAGEMENT_SUBMAP_STITCHING_H_

#include <map>
#include <memory>
#include <random>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/common/plane_type.h"
#include "panoptic_mapping/map/submap.h"

#include "voxgraph/frontend/submap_collection/bounding_box.h"

namespace panoptic_mapping {
// TODO(supernlogn): see if this can occupy less memory
struct PointIndexType {
  PointIndexType(const voxblox::BlockIndex b_i, const int l_i)
      : block_index(b_i), linear_index(l_i) {}
  voxblox::BlockIndex block_index;
  int linear_index;
};

class SubmapStitching {
 public:
  typedef int ClassID;
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;
    float z_threshold = 0.15;
    float xy_threshold = 0.01;
    int max_walls = 3;
    int max_floors = 1;
    int max_ceilings = 1;
    float normal_cluster_threshold = 0.1;     // no-unit
    float position_cluster_threshold = 0.05;  // [m]
    float max_outlier_percentage = 0.70;
    float satisfying_outlier_percent = 0.70;
    int ransac_num_iterations = 1000;
    uint_fast32_t random_generator_seed = 100;
    Config() { setConfigName("SubmapStitching"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };
  explicit SubmapStitching(const Config& config);
  virtual ~SubmapStitching() = default;

  void processSubmap(Submap* s);

  bool stitch(const Submap& SubmapA, Submap* submapB);
  int findNeighboors(const Submap& s, std::vector<int>* neighboor_ids);
  /**
   * @brief Get the Max Num Planes Per class id
   *
   * @param class_id the class of the object
   * @return int maximum number of planes
   */
  int getMaxNumPlanesPerType(ClassID class_id) const;
  // float SubmapStitching::getClusteringThresholdForClass(const ClassID
  // class_id) const;
  static const std::array<ClassID, 3> getBackgroundClassIDS() {
    return {0, 1, 2};
  }
  static void set_seed(const int new_seed_num) {
    seed_num_ = new_seed_num;
    random_number_generator_.seed(new_seed_num);
  }
  /**
   * @brief Get the random sample in the interval [min, max)
   *
   * @param min the minimum value (included) of the sample interval
   * @param max the maximum value (excluded of the sample interval
   * @return uint_fast32_t the sample
   */
  static uint_fast32_t getSample(const int min, const int max) {
    // assert(min >= 0 && max >= 0);
    // assert(max >= min);
    uint_fast32_t min_max_ = min + max;
    uint_fast32_t s = random_number_generator_();
    return (s % min_max_) + static_cast<uint_fast32_t>(min);
  }

 protected:
  struct c_info_t {
    int n;
    Point point;
    Point normal;
    c_info_t(int a0, Point a1, Point a2);
    static bool compDesc(const c_info_t& l, const c_info_t& r) {
      return l.n > r.n;
    }
  };
  // for individual submap
  void findSubmapPlanes(
      classToPlanesType* result, const voxblox::MeshLayer& meshLayer,
      const std::map<SubmapStitching::ClassID, std::vector<PointIndexType> >&
          filtered_class_indices);
  bool planeRansac(std::vector<PlaneType>* merged_result,
                   const voxblox::MeshLayer& mesh_layer,
                   const std::vector<PointIndexType>& p_indices,
                   const int num_iterations, const int max_num_planes,
                   const ClassID class_id);
  bool planeRansacSimple(std::vector<PlaneType>* merged_result,
                         const voxblox::MeshLayer& mesh_layer,
                         const std::vector<PointIndexType>& p_indices,
                         const int num_iterations, const int max_num_planes,
                         const ClassID class_id);
  std::vector<Eigen::Hyperplane<float, 3> > ransacSample(
      const std::vector<const Point*>& mesh_points,
      const std::vector<const Point*>& mesh_normals, const int max_num_planes,
      const ClassID class_id) const;
  Eigen::Hyperplane<float, 3> ransacSampleSingle(
      const std::vector<const Point*>& mesh_points, const int max_num_planes,
      const ClassID class_id) const;
  void applyClassPreFilter(
      std::map<SubmapStitching::ClassID, std::vector<PointIndexType> >* ret,
      const voxblox::MeshLayer& meshLayer, const ClassLayer& class_layer);
  Eigen::Hyperplane<float, 3> createPlaneFrom3Points(
      const Point& p1, const Point& p2, const Point& p3,
      const Eigen::Vector3f& class_dir) const;
  // return the number of outlier points given a set of planes and the whole
  // point set.
  int ransacCheck(const std::vector<Eigen::Hyperplane<float, 3> >& hyperplanes,
                  const std::vector<const Point*>& mesh_points);
  int ransacCheckSingle(const Eigen::Hyperplane<float, 3>& hyperplane,
                        const std::vector<const Point*>& mesh_points) const;
  std::pair<const Point*, const Point*> getPointAndNormalFromPointIndex(
      const PointIndexType& p_idx, const voxblox::MeshLayer& mesh_layer) const;
  const Point& getNormalFromPointIndex(
      const PointIndexType& p_idx, const voxblox::MeshLayer& mesh_layer) const;

  const Point& getPointFromPointIndex(
      const PointIndexType& p_idx, const voxblox::MeshLayer& mesh_layer) const;
  // for stitching submap
  void matchNeighboorPlanes(const Submap& SubmapA, const Submap& SubmapB,
                            const std::vector<int>& neighboor_ids);
  void mini_clustering(std::vector<Eigen::Hyperplane<float, 3> >* major_planes,
                       const std::vector<Point>& point_set,
                       const std::vector<Point>& normals_set,
                       const int num_clustered_planes,
                       const float threshold) const;

 private:
  const Config config_;
  std::map<int, int> submap_neighboors;
  // random number generator and its seed used for ransac
  std::shared_ptr<std::map<int, classToPlanesType> >
      submap_id_to_class_to_planes_;
  static int seed_num_;
  static std::mt19937 random_number_generator_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_MANAGEMENT_SUBMAP_STITCHING_H_
