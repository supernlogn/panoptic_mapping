#ifndef PANOPTIC_MAPPING_MAP_MANAGEMENT_PLANE_COLLECTION_H_
#define PANOPTIC_MAPPING_MAP_MANAGEMENT_PLANE_COLLECTION_H_

#include <map>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/Shape_detection.h>
#include <CGAL/property_map.h>
#include <Eigen/Geometry>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <voxgraph/frontend/plane_collection/plane_type.h>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/map/submap.h"

#include "voxgraph/frontend/submap_collection/bounding_box.h"

namespace panoptic_mapping {

// Type declarations for cgal plane extractor
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef std::pair<Kernel::Point_3, Kernel::Vector_3> Point_with_normal;
typedef std::vector<Point_with_normal> Pwn_vector;
typedef CGAL::First_of_pair_property_map<Point_with_normal> Point_map;
typedef CGAL::Second_of_pair_property_map<Point_with_normal> Normal_map;
typedef CGAL::Shape_detection::Efficient_RANSAC_traits<Kernel, Pwn_vector,
                                                       Point_map, Normal_map>
    Traits;
typedef CGAL::Shape_detection::Efficient_RANSAC<Traits> Efficient_ransac;
typedef CGAL::Shape_detection::Plane<Traits> Cgal_plane_type;

// TODO(supernlogn): see if this can occupy less memory
struct PointIndexType {
  PointIndexType(const voxblox::BlockIndex b_i, const int l_i)
      : block_index(b_i), linear_index(l_i) {}
  voxblox::BlockIndex block_index;
  int linear_index;
};

using classToPlanesType = voxgraph::classToPlanesType;
using PlaneType = voxgraph::PlaneType;
using BoundingBoxType = voxgraph::BoundingBoxType;

class PlaneCollection {
 public:
  typedef int ClassID;
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 1;
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
    // Sets probability to miss the largest primitive at each iteration.
    double ransac_probability = 0.05;
    // Detect shapes with at least so many points.
    size_t ransac_min_points = size_t(2000u);
    // Sets maximum Euclidean distance between a point and a shape.
    double ransac_epsilon = 0.05;
    // Sets maximum Euclidean distance between points to be clustered.
    double ransac_cluster_epsilon = 0.05;
    // Sets maximum normal deviation.
    // 0.9 < dot(surface_normal, point_normal);
    double ransac_normal_threshold = 0.1;
    uint_fast32_t random_generator_seed = 100;
    // if not empty planes found are visualized
    // after processing a submap
    std::string plane_visualization_topic = "";
    Config() { setConfigName("PlaneCollection"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };
  explicit PlaneCollection(const Config& config);
  virtual ~PlaneCollection() = default;

  void processSubmap(Submap* s);
  void visualizePlanesOfClasses(const classToPlanesType& class_to_planes);
  void visualizePlanesOfClass(const ClassID class_id,
                              const std::vector<PlaneType>& planes);
  // void publishNormal(const PlaneType& plane, const int marker_id,
  //                    const int class_id);
  /**
   * @brief Get the Max Num Planes Per class id
   *
   * @param class_id the class of the object
   * @return int maximum number of planes
   */
  int getMaxNumPlanesPerType(ClassID class_id) const;

  const classToPlanesType& getPlanesForSubmapID(const int submap_id) const {
    return submap_id_to_class_to_planes_->at(submap_id);
  }

  std::vector<panoptic_mapping_msgs::PlaneType> getPlaneMessagesForSubmapID(
      const int submap_id) const {
    std::vector<panoptic_mapping_msgs::PlaneType> ret;
    if (submap_id_to_class_to_planes_->find(submap_id) !=
        submap_id_to_class_to_planes_->end()) {
      for (const auto& class_plane_pair :
           submap_id_to_class_to_planes_->at(submap_id)) {
        for (const auto& plane : class_plane_pair.second) {
          ret.push_back(plane.toPlaneTypeMsg());
        }
      }
    }
    return ret;
  }
  // float PlaneCollection::getClusteringThresholdForClass(const ClassID
  // class_id) const;
  static constexpr std::array<ClassID, 3> getBackgroundClassIDS() {
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
      classToPlanesType* result, const voxblox::MeshLayer& mesh_layer,
      const Transformation T_mid_pose,
      const std::map<PlaneCollection::ClassID, std::vector<PointIndexType>>&
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
  std::vector<Eigen::Hyperplane<float, 3>> ransacSample(
      const std::vector<const Point*>& mesh_points,
      const std::vector<const Point*>& mesh_normals, const int max_num_planes,
      const ClassID class_id) const;
  Eigen::Hyperplane<float, 3> ransacSampleSingle(
      const std::vector<const Point*>& mesh_points, const int max_num_planes,
      const ClassID class_id) const;
  void applyClassPreFilter(
      std::map<PlaneCollection::ClassID, std::vector<PointIndexType>>* ret,
      const TsdfLayer& tsdf_layer, const voxblox::MeshLayer& meshLayer,
      const ClassLayer& class_layer);
  Eigen::Hyperplane<float, 3> createPlaneFrom3Points(
      const Point& p1, const Point& p2, const Point& p3,
      const Eigen::Vector3f& class_dir) const;
  // return the number of outlier points given a set of planes and the whole
  // point set.
  int ransacCheck(const std::vector<Eigen::Hyperplane<float, 3>>& hyperplanes,
                  const std::vector<const Point*>& mesh_points);
  int ransacCheckSingle(const Eigen::Hyperplane<float, 3>& hyperplane,
                        const std::vector<const Point*>& mesh_points) const;
  std::pair<const Point*, const Point*> getPointAndNormalFromPointIndex(
      const PointIndexType& p_idx, const voxblox::MeshLayer& mesh_layer) const;
  const Point& getNormalFromPointIndex(
      const PointIndexType& p_idx, const voxblox::MeshLayer& mesh_layer) const;

  const Point& getPointFromPointIndex(
      const PointIndexType& p_idx, const voxblox::MeshLayer& mesh_layer) const;
  void mini_clustering(std::vector<Eigen::Hyperplane<float, 3>>* major_planes,
                       const std::vector<Point>& point_set,
                       const std::vector<Point>& normals_set,
                       const int num_clustered_planes,
                       const float threshold) const;
  bool cgalExtractPlane(std::vector<PlaneType>* merged_result,
                        const voxblox::MeshLayer& mesh_layer,
                        const Transformation& T_mid_pose,
                        const std::vector<PointIndexType>& p_indices,
                        const int num_iterations, const int max_num_planes,
                        const int class_id);
  void cgalSetPlaneExtractor();

 private:
  const Config config_;
  std::map<int, int> submap_neighboors;
  // random number generator and its seed used for ransac
  std::shared_ptr<std::map<int, classToPlanesType>>
      submap_id_to_class_to_planes_;
  static int seed_num_;
  static std::mt19937 random_number_generator_;
  Efficient_ransac::Parameters shape_detection_parameters_;
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_visualizer_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_MANAGEMENT_PLANE_COLLECTION_H_
