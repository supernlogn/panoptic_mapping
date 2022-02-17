#include "panoptic_mapping/map_management/submap_stitching.h"

#include <map>
#include <random>
#include <set>
#include <utility>
#include <vector>

#include "panoptic_mapping/common/index_getter.h"

namespace panoptic_mapping {

int SubmapStitching::seed_num_ = 2;
std::mt19937 SubmapStitching::random_number_generator_;

void SubmapStitching::Config::checkParams() const {
  checkParamGT(z_threshold, 0.0f, "z_threshold");
  checkParamGT(xy_threshold, 0.0f, "xy_threshold");
  checkParamGT(max_walls, 0, "max_walls");
  checkParamGT(max_floors, 0, "max_floors");
  checkParamGT(max_ceilings, 0, "max_ceilings");
  checkParamGT(ransac_num_iterations, 0, "ransac_num_iterations");
}

void SubmapStitching::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("z_threshold", &z_threshold);
  setupParam("xy_threshold", &xy_threshold);
  setupParam("max_walls", &max_walls);
  setupParam("max_floors", &max_floors);
  setupParam("max_ceilings", &max_ceilings);
  setupParam("ransac_num_iterations", &ransac_num_iterations);
  setupParam("max_outlier_percentage", &max_outlier_percentage);
  setupParam("satisfying_outlier_percent", &satisfying_outlier_percent);
}

SubmapStitching::SubmapStitching(const Config& config)
    : config_(config.checkValid()) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
  set_seed(config_.random_generator_seed);
}

void SubmapStitching::processSubmap(const Submap& s) {
  std::map<SubmapStitching::ClassID, std::vector<PointIndexType> >
      filtered_class_indices;
  applyClassPreFilter(&filtered_class_indices, s.getMeshLayer(),
                      s.getClassLayer());
  s.getIsoSurfacePoints();
  findSubmapPlanes(s.getMeshLayer(), filtered_class_indices);
}

void SubmapStitching::findSubmapPlanes(
    const voxblox::MeshLayer& mesh_layer,
    const std::map<SubmapStitching::ClassID, std::vector<PointIndexType> >&
        filtered_class_indices) {
  const auto& background_class_ids = getBackgroundClassIDS();
  std::map<ClassID, std::vector<PlaneType> > class_id_to_planes;
  for (const auto& p_indices_pair : filtered_class_indices) {
    ClassID class_id = p_indices_pair.first;
    class_id_to_planes[class_id] = std::vector<PlaneType>();
    const int max_num_planes = getMaxNumPlanesPerType(class_id);
    planeRansac(&class_id_to_planes[class_id], mesh_layer,
                p_indices_pair.second, config_.ransac_num_iterations,
                max_num_planes, class_id);
  }
}

bool SubmapStitching::planeRansac(std::vector<PlaneType>* merged_result,
                                  const voxblox::MeshLayer& mesh_layer,
                                  const std::vector<PointIndexType>& p_indices,
                                  const int num_iterations,
                                  const int max_num_planes,
                                  const ClassID class_id) {
  std::vector<Eigen::Hyperplane<float, 3> > best_result;
  std::vector<std::pair<ClassID, BoundingBoxType> > best_bounding_boxes = {};
  size_t num_points = p_indices.size();
  size_t num_outliers = num_points;
  bool is_satisfying_result = false;
  const size_t satisfying_num_outliers =
      config_.satisfying_outlier_percent * num_points;

  // create vector with all points
  // TODO(supernlogn): This is a completely indepedent
  // procedure. Maybe put it somewhere else
  std::vector<const Point*> mesh_points(p_indices.size());
  std::vector<const Point*> mesh_normals(p_indices.size());
  {
    int i = 0;
    for (const auto& p_idx : p_indices) {
      const auto& p_n = getPointAndNormalFromPointIndex(p_idx, mesh_layer);
      mesh_points[i] = p_n.first;
      mesh_normals[i] = p_n.second;
      ++i;
    }
  }
  for (int i = 0; i < num_iterations; ++i) {
    const auto& hyperplanes =
        ransacSample(mesh_points, mesh_normals, max_num_planes, class_id);
    int temp_num_outliers = ransacCheck(hyperplanes, mesh_points);
    if (temp_num_outliers < num_outliers) {
      best_result = hyperplanes;
      num_outliers = temp_num_outliers;
    }
    if (temp_num_outliers < satisfying_num_outliers) {
      is_satisfying_result = true;
      break;
    }
  }
  if (is_satisfying_result) {
    LOG_IF(INFO, config_.verbosity >= 3) << "Num outliers result is satisfying";
  } else {
    LOG_IF(INFO, config_.verbosity >= 3)
        << "Num outliers result is not satisfying. Exhausted num of iterations";
  }
  LOG_IF(INFO, config_.verbosity >= 2) << "num_outliers=" << num_outliers;
  bool is_ok = num_outliers < config_.max_outlier_percentage * num_points;
  if (is_ok) {
    // create plane_type from eigen::Hyperplane
    for (const auto& plane : best_result) {
      merged_result->emplace_back(plane, class_id);
      PlaneType& full_plane = merged_result->back();
      full_plane.createPlaneSegmentAaBb(mesh_points,
                                        config_.position_cluster_threshold);
      // create bounding boxes around planes
    }
  }
  return is_ok;
}

std::vector<Eigen::Hyperplane<float, 3> > SubmapStitching::ransacSample(
    const std::vector<const Point*>& mesh_points,
    const std::vector<const Point*>& mesh_normals, const int max_num_planes,
    const ClassID class_id) const {
  // get 3 random points
  int sample_maximum = mesh_points.size();
  std::vector<Point> point_subset(max_num_planes * 3);
  std::vector<Point> normals_subset(max_num_planes * 3);
  for (int i = 0; i < 3 * max_num_planes; ++i) {
    const auto s = getSample(0, sample_maximum);
    point_subset[i] = *mesh_points[s];
    normals_subset[i] = *mesh_normals[s];
  }
  // const Point class_dir = getDirectionOfClassID(class_id);
  // cluster major plane
  const float threshold = config_.position_cluster_threshold;
  std::vector<Eigen::Hyperplane<float, 3> > ret;
  mini_clustering(&ret, point_subset, normals_subset, max_num_planes,
                  threshold);
  return ret;
}

// void rejectAlgo(std::vector<Eigen::Hyperplane<float, 3> > * major_planes,
//                      const std::vector<Point> &point_set,
//                      const std::vector<Point> & normals_set,
//                      const float threshold) {
//   const size_t N = point_set.size();
//   // initialize reject, means, nums matrices
//   bool reject_mtx[N];
//   Point mean_points[N];
//   Point mean_normals[N];
//   int nums[N];
//   const Point p_zero = Point::Zero(3);
//   for(int i = 0; i <N;++i){
//     reject_mtx[i] = false;
//     mean_points[i] = p_zero;
//     mean_normals[i] = p_zero;
//     nums[i] = 1;
//   }
//   // start rejecting planes
//   for (int i = 0; i < N; ++i) {
//     if (reject_mtx[i]) {
//       continue;
//     }
//     mean_points[i] += point_set[i];
//     mean_normals[i] += normals_set[i];
//     for (int j = i+1; j < N; ++j) {
//       if (reject_mtx[j]) {
//         continue;
//       }
//       float dist = PlaneType::distFunc2(point_set[i], normals_set[i],
//       point_set[j], normals_set[j]); if (dist < threshold) {
//         reject_mtx[j] = true;
//         mean_points[i] += point_set[j];
//         mean_normals[i] += normals_set[j];
//         nums[i]++;
//       }
//     }
//     mean_points[i] /= nums[i];
//     mean_normals[i] /= nums[i];
//   }

//   for(int i = 0; i < N; ++i) {

//   }
// }

void SubmapStitching::mini_clustering(
    std::vector<Eigen::Hyperplane<float, 3> >* major_planes,
    const std::vector<Point>& point_set, const std::vector<Point>& normals_set,
    const int num_clustered_planes, const float threshold) const {
  /* naive distance matrix computation */
  const Point* p_data = point_set.data();
  const Point* n_data = normals_set.data();
  const size_t N = point_set.size();
  /* get row with the smallest sum */
  int classes[N];
  for (int i = 0; i < N; ++i) {
    classes[i] = i;
  }
  for (int i = 0; i < N; ++i) {
    for (int j = i + 1; j < N; ++j) {
      const float dist =
          PlaneType::distFunc2(p_data[i], n_data[i], p_data[j], n_data[j]);
      if (dist < threshold) {
        classes[j] = classes[i];
      }
    }
  }
  std::set<int> distinct_classes;
  for (int i = 0; i < N; ++i) {
    distinct_classes.insert(classes[i]);
  }
  const Point zero_point = Point::Zero(3);
  for (int cluster_class : distinct_classes) {
    Point mean_point = zero_point;
    Point mean_normal = zero_point;
    int n = 0;
    for (int i = 0; i < N; ++i) {
      if (classes[i] == cluster_class) {
        mean_point += p_data[i];
        mean_normal += n_data[i];
        ++n;
      }
    }
    mean_point /= n;
    mean_normal /= n;
    major_planes->emplace_back(mean_normal, mean_point);
  }
}

int SubmapStitching::ransacCheck(
    const std::vector<Eigen::Hyperplane<float, 3> >& hyperplanes,
    const std::vector<const Point*>& mesh_points) {
  int num_outliers = 0;
  for (const auto p : mesh_points) {
    // check if it is outlier
    bool is_outlier = true;
    for (const auto& plane : hyperplanes) {
      // TODO(supernlogn): see that thresholds are different for each class
      if (plane.absDistance(*p) < config_.position_cluster_threshold) {
        is_outlier = false;
        break;
      }
    }
    if (is_outlier) {
      ++num_outliers;
    }
  }
  return num_outliers;
}

Eigen::Hyperplane<float, 3> SubmapStitching::createPlaneFrom3Points(
    const Point& p1, const Point& p2, const Point& p3,
    const Eigen::Vector3f& class_dir) const {
  Eigen::Vector3f p12 = p2 - p1;
  Eigen::Vector3f p13 = p3 - p1;
  Eigen::Vector3f normal = p12.cross(p13);
  // we want all normals to have the same orientation
  // so that we can be able to compare them
  if (normal.dot(class_dir) < 0) {
    normal = -normal;
  }
  return Eigen::Hyperplane<float, 3>(normal, p1);
}

void SubmapStitching::applyClassPreFilter(
    std::map<SubmapStitching::ClassID, std::vector<PointIndexType> >* ret,
    const voxblox::MeshLayer& mesh_layer, const ClassLayer& classLayer) {
  // assign all background class IDs a zero-element vector
  const auto background_class_ids = getBackgroundClassIDS();
  for (ClassID class_id : background_class_ids) {
    ret->insert({class_id, std::vector<PointIndexType>()});
  }
  voxblox::BlockIndexList mesh_indices;
  mesh_layer.getAllUpdatedMeshes(
      &mesh_indices);  // TODO(supernlogn): See if this is not needed
  LOG(INFO) << "mesh_indices.size() =" << mesh_indices.size();
  // iterate over all submap blocks
  for (const voxblox::BlockIndex& block_index : mesh_indices) {
    voxblox::Mesh::ConstPtr mesh = mesh_layer.getMeshPtrByIndex(block_index);
    panoptic_mapping::ClassBlock::ConstPtr class_block =
        classLayer.getBlockConstPtrByIndex(block_index);
    // iterate over all indices of this block
    const size_t num_vertices = mesh->vertices.size();
    for (size_t i = 0u; i < num_vertices; ++i) {
      ClassID class_id = class_block->getVoxelByLinearIndex(i).getBelongingID();
      // add this point index to its class list
      if (std::find(background_class_ids.begin(), background_class_ids.end(),
                    class_id) != background_class_ids.end()) {
        ret->at(class_id).emplace_back(block_index, i);
      }
    }
  }
}

// for stitching submap
bool SubmapStitching::stitch(const Submap& SubmapA, Submap* submapB) {}

int SubmapStitching::findNeighboors(const Submap& s,
                                    std::vector<int>* neighboor_ids) {}
void SubmapStitching::matchNeighboorPlanes(
    const Submap& SubmapA, const Submap& SubmapB,
    const std::vector<int>& neighboor_ids) {}

// general
int SubmapStitching::getMaxNumPlanesPerType(ClassID class_id) const {
  // TODO(supernlogn): define them arbitary
  const ClassID ceiling_id = 0;
  const ClassID floor_id = 1;
  const ClassID wall_id = 2;
  switch (class_id) {
    case ceiling_id:
      return config_.max_ceilings;
    case floor_id:
      return config_.max_floors;
    case wall_id:
      return config_.max_walls;
    default:
      LOG(ERROR) << "No background class with ID: " << class_id;
      return 0;
  }
  return 0;
}

std::pair<const Point*, const Point*>
SubmapStitching::getPointAndNormalFromPointIndex(
    const PointIndexType& p_idx, const voxblox::MeshLayer& mesh_layer) const {
  const auto& block = mesh_layer.getMeshPtrByIndex(p_idx.block_index);
  return std::make_pair(&block->vertices[p_idx.linear_index],
                        &block->normals[p_idx.linear_index]);
}

const Point& SubmapStitching::getNormalFromPointIndex(
    const PointIndexType& p_idx, const voxblox::MeshLayer& mesh_layer) const {
  return mesh_layer.getMeshPtrByIndex(p_idx.block_index)
      ->normals[p_idx.linear_index];
}

const Point& SubmapStitching::getPointFromPointIndex(
    const PointIndexType& p_idx, const voxblox::MeshLayer& mesh_layer) const {
  return mesh_layer.getMeshPtrByIndex(p_idx.block_index)
      ->vertices[p_idx.linear_index];
}

float getMeshNormals(Submap* submap) {
  auto mesh_layer = submap->getMeshLayerPtr();
  voxblox::BlockIndexList mesh_indices;
  mesh_layer->getAllUpdatedMeshes(
      &mesh_indices);  // TODO(supernlogn): See if this is not needed
  LOG(INFO) << "mesh_indices.size() =" << mesh_indices.size();
  voxblox::Pointcloud floor_vertices;
  voxblox::Point up(0, 1.0, 0.0);
  float up_threshold = 0.8;
  // const int class_id = submap->getClassID();
  const auto& classLayer = submap->getClassLayer();
  for (const voxblox::BlockIndex& block_index : mesh_indices) {
    voxblox::Mesh::ConstPtr mesh = mesh_layer->getMeshPtrByIndex(block_index);
    panoptic_mapping::ClassBlock::ConstPtr class_block =
        classLayer.getBlockConstPtrByIndex(block_index);
    // TODO(supernlogn) clustering can be done with label also
    for (size_t i = 0u; i < mesh->vertices.size(); ++i) {
      const auto pn = mesh->normals[i];
      const auto& class_voxel = class_block->getVoxelByLinearIndex(i);
      std::cout << pn << ";";
      float dot_prod = pn.dot(up);
      if (dot_prod > up_threshold) {
        floor_vertices.push_back(mesh->vertices[i]);
      }
    }
    std::cout << '\n';
    std::cout << "mesh->vertices.size() = " << mesh->vertices.size() << '\n';
  }

  double mean_vertice_y = 0.0;
  for (const auto vert : floor_vertices) {
    mean_vertice_y += vert.y();
  }
  if (floor_vertices.size() > 0) {
    mean_vertice_y /= floor_vertices.size();
  } else {
    LOG(WARNING) << "No floor vertices pointing up";
  }

  return static_cast<float>(mean_vertice_y);
}

}  // namespace panoptic_mapping
