#include "panoptic_mapping/map_management/submap_stitching.h"

#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <unordered_map>
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
  setupParam("publish_bboxes_topic", &publish_bboxes_topic);
}

SubmapStitching::SubmapStitching(const Config& config)
    : config_(config.checkValid()), nh_("") {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
  set_seed(config_.random_generator_seed);
  submap_id_to_class_to_planes_ =
      std::make_shared<std::map<int, classToPlanesType>>(
          std::map<int, classToPlanesType>());
  if (!config_.publish_bboxes_topic.empty()) {
    bboxes_publisher_ = nh_.advertise<visualization_msgs::Marker>(
        config_.publish_bboxes_topic, 100);
  }
}

void SubmapStitching::processSubmap(Submap* s) {
  std::map<SubmapStitching::ClassID, std::vector<size_t>>
      filtered_class_indices;
  assert(s->hasClassLayer());

  applyClassPreFilter(&filtered_class_indices, s->getMeshLayer(),
                      s->getClassLayer());
  // s->getIsoSurfacePoints();
  classToPlanesType class_id_to_planes;
  findSubmapPlanes(&class_id_to_planes, s->getMeshLayer(),
                   s->getIsoSurfacePoints(), filtered_class_indices);
  std::stringstream ss;
  ss << "class -- num_points -- num_planes";
  for (const auto& p : class_id_to_planes) {
    ss << "\n"
       << p.first << "--" << filtered_class_indices.at(p.first).size() << "--"
       << p.second.size();
  }
  LOG(INFO) << "\n" << ss.str();
  submap_id_to_class_to_planes_->insert({s->getID(), class_id_to_planes});
  publish_new_bboxes(class_id_to_planes);
}

void SubmapStitching::findSubmapPlanes(
    classToPlanesType* result, const voxblox::MeshLayer& mesh_layer,
    const std::vector<IsoSurfacePoint>& iso_surface_points,
    const std::map<SubmapStitching::ClassID, std::vector<size_t>>&
        filtered_class_indices) {
  CHECK_NOTNULL(result);
  LOG_IF(INFO, config_.verbosity >= 3) << "Entered findSubmapPlanes";
  for (const auto& p_indices_pair : filtered_class_indices) {
    ClassID class_id = p_indices_pair.first;
    result->insert({class_id, std::vector<PlaneType>()});
    const int max_num_planes = getMaxNumPlanesPerType(class_id);
    LOG_IF(INFO, config_.verbosity >= 3)
        << "Calling planeRansac for class" << class_id;
    if (p_indices_pair.second.size() > 0) {
      planeRansacSimple(&result->at(class_id), iso_surface_points,
                        p_indices_pair.second, config_.ransac_num_iterations,
                        max_num_planes, class_id);
    } else {
      LOG_IF(INFO, config_.verbosity >= 4) << "skipping class with no indices";
    }
  }
}

bool SubmapStitching::planeRansac(std::vector<PlaneType>* merged_result,
                                  const voxblox::MeshLayer& mesh_layer,
                                  const std::vector<PointIndexType>& p_indices,
                                  const int num_iterations,
                                  const int max_num_planes,
                                  const ClassID class_id) {
  std::vector<Eigen::Hyperplane<float, 3>> best_result;
  std::vector<std::pair<ClassID, BoundingBoxType>> best_bounding_boxes = {};
  size_t num_points = p_indices.size();
  size_t num_outliers = num_points;
  bool is_satisfying_result = false;
  const size_t satisfying_num_outliers =
      config_.satisfying_outlier_percent * num_points;
  LOG_IF(INFO, config_.verbosity >= 1)
      << "start planeRansac for " << num_points << " points";
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
    LOG_IF(INFO, config_.verbosity >= 3)
        << "Planeransac: Initialized mesh_points and mesh_normals pointers";
  }
  // run ransac's main iteration
  for (int i = 0; i < num_iterations; ++i) {
    const auto& hyperplanes =
        ransacSample(mesh_points, mesh_normals, max_num_planes, class_id);
    int temp_num_outliers = ransacCheck(hyperplanes, mesh_points);
    if (temp_num_outliers < num_outliers) {
      best_result = hyperplanes;
      num_outliers = temp_num_outliers;
      if (temp_num_outliers < satisfying_num_outliers) {
        is_satisfying_result = true;
        break;
      }
    }
  }

  if (is_satisfying_result) {
    LOG_IF(INFO, config_.verbosity >= 3) << "Num outliers result is satisfying";
  } else {
    LOG_IF(WARNING, config_.verbosity >= 3)
        << "Num outliers result is not satisfying. Exhausted num of iterations";
  }
  LOG_IF(INFO, config_.verbosity >= 2)
      << "num_outliers/satisfying_num_outliers/num_points=" << num_outliers
      << "/" << satisfying_num_outliers << "/" << num_points;
  bool is_ok = num_outliers < config_.max_outlier_percentage * num_points;
  if (is_ok) {
    // create plane_type from eigen::Hyperplane
    LOG_IF(INFO, config_.verbosity >= 3)
        << "Planeransac: Create plane from eigen::Hyperplane";
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

bool SubmapStitching::planeRansacSimple(
    std::vector<PlaneType>* merged_result,
    const std::vector<IsoSurfacePoint>& iso_surface_points,
    const std::vector<size_t>& p_indices, const int num_iterations,
    const int max_num_planes, const ClassID class_id) {
  Eigen::Hyperplane<float, 3> best_result;
  std::vector<std::pair<ClassID, BoundingBoxType>> best_bounding_boxes = {};
  size_t num_points = p_indices.size();
  size_t num_outliers = num_points;
  LOG_IF(INFO, config_.verbosity >= 1)
      << "start planeRansac for " << num_points << " points";

  // create vector with all points
  // TODO(supernlogn): This is a completely indepedent
  // procedure. Maybe put it somewhere else
  std::vector<const Point*> mesh_points(num_points);
  {
    int i = 0;
    for (const auto& p_idx : p_indices) {
      mesh_points[i] = &iso_surface_points[p_idx].position;
      ++i;
    }
    LOG_IF(INFO, config_.verbosity >= 3)
        << "Planeransac: Initialized mesh_points pointers";
  }
  for (int k = 0; k < max_num_planes; ++k) {
    int skipped = 0;
    // run ransac's main iteration
    num_outliers = mesh_points.size();
    for (int i = 0; i < num_iterations; ++i) {
      const auto& hyperplane =
          ransacSampleSingle(mesh_points, max_num_planes, class_id);
      if (hyperplane.normal().squaredNorm() < 1e-8) {
        ++skipped;
        continue;
      }
      int temp_num_outliers = ransacCheckSingle(hyperplane, mesh_points);
      if (temp_num_outliers < num_outliers) {
        best_result = hyperplane;
        num_outliers = temp_num_outliers;
      }
    }
    if (skipped == 1000) {
      LOG(ERROR) << "Skipped all iterations because of very small norm";
    }
    LOG_IF(WARNING, skipped > 100)
        << "Skipped " << skipped << " iterations with very small norm";
    // create result
    // and bounding boxes around planes
    float k_term = (max_num_planes - k) / static_cast<float>(max_num_planes);
    if (num_outliers <
        config_.max_outlier_percentage * k_term * mesh_points.size()) {
      merged_result->emplace_back(best_result, class_id);
      PlaneType& full_plane = merged_result->back();
      full_plane.createPlaneSegmentAaBb(mesh_points,
                                        config_.position_cluster_threshold);
      // eliminate points for the next plane
      for (int i = mesh_points.size() - 1; i >= 0; --i) {
        if (best_result.absDistance(*mesh_points[i]) <
            config_.position_cluster_threshold) {
          mesh_points.erase(mesh_points.begin() + i);
        }
      }
    }
    // if enough points iterate
    if (mesh_points.size() < 10) {
      break;
    }
  }

  LOG_IF(INFO, config_.verbosity >= 2)
      << "num_outliers/num_points=" << mesh_points.size() << "/" << num_points;
  bool is_ok = num_outliers < config_.max_outlier_percentage * num_points;
  LOG_IF(INFO, config_.verbosity >= 3) << "PlaneransacSimple: is_ok =" << is_ok;
  return is_ok;
}

std::vector<Eigen::Hyperplane<float, 3>> SubmapStitching::ransacSample(
    const std::vector<const Point*>& mesh_points,
    const std::vector<const Point*>& mesh_normals, const int max_num_planes,
    const ClassID class_id) const {
  // get 3 random points
  int sample_maximum = mesh_points.size();
  LOG_IF(INFO, config_.verbosity >= 5)
      << "ransacSample for " << sample_maximum << " points";
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
  std::vector<Eigen::Hyperplane<float, 3>> ret;
  // mini_clustering(&ret, point_subset, normals_subset, max_num_planes,
  //                 threshold);
  Eigen::Vector3f class_dir;
  if (class_id == 0) {
    class_dir = Point(0.0, 0.0, -1.0);
  } else if (class_id == 1) {
    class_dir = Point(0.0, 0.0, 1.0);
  } else {
    class_dir = Point(0.5, 0.5, 0.5);
  }
  for (int i = 0; i < max_num_planes; ++i) {
    const int b_i = 3 * i;
    const auto& hplane =
        createPlaneFrom3Points(point_subset[b_i], point_subset[b_i + 1],
                               point_subset[b_i + 2], class_dir);
    ret.push_back(hplane);
  }
  return ret;
}

Eigen::Hyperplane<float, 3> SubmapStitching::ransacSampleSingle(
    const std::vector<const Point*>& mesh_points, const int max_num_planes,
    const ClassID class_id) const {
  const int sample_maximum = mesh_points.size();
  const auto s1 = getSample(0, sample_maximum);
  auto s2 = getSample(0, sample_maximum);
  int i = 0;
  while (s2 == s1 && i < 10) {
    s2 = getSample(0, sample_maximum);
    ++i;
  }
  auto s3 = getSample(0, sample_maximum);
  i = 0;
  while ((s3 == s1 || s3 == s2) && i < 10) {
    s3 = getSample(0, sample_maximum);
    ++i;
  }
  const Point* p1 = mesh_points[s1];
  const Point* p2 = mesh_points[s2];
  const Point* p3 = mesh_points[s3];
  Eigen::Vector3f class_dir;
  if (class_id == 0) {
    class_dir = Point(0.0, 0.0, -1.0);
  } else if (class_id == 1) {
    class_dir = Point(0.0, 0.0, 1.0);
  } else {
    class_dir = Point(0.5, 0.5, 0.5);
  }
  return createPlaneFrom3Points(*p1, *p2, *p3, class_dir);
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
    std::vector<Eigen::Hyperplane<float, 3>>* major_planes,
    const std::vector<Point>& point_set, const std::vector<Point>& normals_set,
    const int num_clustered_planes, const float threshold) const {
  /* naive distance matrix computation */
  const Point* p_data = point_set.data();
  const Point* n_data = normals_set.data();
  const size_t N = point_set.size();
  /* get row with the smallest sum */
  int mst_classes[N];
  for (int i = 0; i < N; ++i) {
    mst_classes[i] = i;
  }
  for (int i = 0; i < N; ++i) {
    for (int j = i + 1; j < N; ++j) {
      const float dist =
          PlaneType::distFunc2(p_data[i], n_data[i], p_data[j], n_data[j]);
      if (dist < threshold) {
        mst_classes[j] = mst_classes[i];
      }
    }
  }
  std::set<int> distinct_classes;
  for (int i = 0; i < N; ++i) {
    distinct_classes.insert(mst_classes[i]);
  }
  const Point zero_point = Point::Zero(3);

  std::vector<c_info_t> mst_classes_info;
  for (int cluster_class : distinct_classes) {
    Point mean_point = zero_point;
    Point mean_normal = zero_point;
    int n = 0;
    for (int i = 0; i < N; ++i) {
      if (mst_classes[i] == cluster_class) {
        mean_point += p_data[i];
        mean_normal += n_data[i];
        ++n;
      }
    }
    mean_point /= n;
    mean_normal /= n;
    mst_classes_info.emplace_back(n, mean_point, mean_normal.normalized());
  }
  std::sort(mst_classes_info.begin(), mst_classes_info.end(),
            c_info_t::compDesc);
  for (int i = 0; i < num_clustered_planes; ++i) {
    const auto& c_i = mst_classes_info[i];
    major_planes->emplace_back(c_i.point, c_i.normal);
  }
}

int SubmapStitching::ransacCheck(
    const std::vector<Eigen::Hyperplane<float, 3>>& hyperplanes,
    const std::vector<const Point*>& mesh_points) {
  LOG_IF(INFO, config_.verbosity >= 5)
      << "Calling ransacCheck for " << hyperplanes.size() << " hyperplanes and"
      << mesh_points.size() << " points";
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

int SubmapStitching::ransacCheckSingle(
    const Eigen::Hyperplane<float, 3>& hyperplane,
    const std::vector<const Point*>& mesh_points) const {
  int num_outliers = 0;
  for (const auto p : mesh_points) {
    // check if it is outlier
    // TODO(supernlogn): see that thresholds are different for each class
    if (hyperplane.absDistance(*p) > config_.position_cluster_threshold) {
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
    std::map<SubmapStitching::ClassID, std::vector<size_t>>* ret,
    const voxblox::MeshLayer& mesh_layer, const ClassLayer& class_layer) {
  CHECK_NOTNULL(ret);
  LOG_IF(INFO, config_.verbosity >= 3)
      << "applying class pre-filter for mesh layer with "
      << mesh_layer.getNumberOfAllocatedMeshes()
      << " allocated meshes and class layer with "
      << class_layer.getNumberOfAllocatedBlocks() << " number of blocks";
  // assign all background class IDs a zero-element vector
  LOG_IF(INFO, config_.verbosity >= 3)
      << "Initializing prefilter for background classes";
  const auto background_class_ids = getBackgroundClassIDS();
  for (ClassID class_id : background_class_ids) {
    ret->insert({class_id, std::vector<size_t>()});
  }
  voxblox::BlockIndexList mesh_indices;
  mesh_layer.getAllUpdatedMeshes(
      &mesh_indices);  // TODO(supernlogn): See if this is not needed
  LOG(INFO) << "mesh_indices.size() =" << mesh_indices.size();
  // iterate over all submap blocks
  size_t counter = 0;
  for (const voxblox::BlockIndex& block_index : mesh_indices) {
    voxblox::Mesh::ConstPtr mesh = mesh_layer.getMeshPtrByIndex(block_index);
    panoptic_mapping::ClassBlock::ConstPtr class_block =
        class_layer.getBlockConstPtrByIndex(block_index);
    int voxel_type = static_cast<int>(class_block->getVoxelType());
    // iterate over all indices of this block
    const size_t num_vertices = mesh->vertices.size();
    LOG(WARNING) << "class_block->num_voxels()/num_vertices = "
                 << class_block->getNumVoxels() << "/" << num_vertices;
    for (size_t i = 0u; i < num_vertices; ++i) {
      ClassID class_id = class_block->getVoxelByLinearIndex(i).getBelongingID();
      // add this point index to its class list
      // if (std::find(background_class_ids.begin(), background_class_ids.end(),
      //               class_id) != background_class_ids.end()) {
      ret->at(class_id).emplace_back(counter + i);
      // }
    }
    counter += num_vertices;
  }
  if (config_.verbosity >= 4) {
    std::stringstream ss;
    for (const auto& pair : *ret) {
      ss << pair.first << "-->" << pair.second.size() << "\n";
    }
    LOG(INFO) << "Finished pre-filtering for the classes-> num point indices"
              << "\n"
              << ss.str();
  }
}

// publish bounding boxes
void SubmapStitching::publish_new_bboxes(
    const classToPlanesType& class_to_planes) {
  if (config_.publish_bboxes_topic.empty()) {
    return;
  }
  for (const auto& pair : class_to_planes) {
    publish_new_bboxes(pair.first, pair.second);
  }
}

void SubmapStitching::publish_new_bboxes(
    const ClassID class_id,
    const std::vector<panoptic_mapping::PlaneType>& planes) {
  static int marker_id = 0;
  for (const auto& plane : planes) {
    auto msg = plane.getVisualizationMsg();
    msg.id = marker_id++;
    if (class_id == 2) {
      msg.color.b = 0.0;
      msg.color.r = 1.0;
    } else if (class_id == 1) {
      /* code */
      msg.color.b = 0.0;
      msg.color.g = 1.0;
    }
    bboxes_publisher_.publish(msg);
  }
}

// for stitching submap
bool SubmapStitching::stitch(const Submap& SubmapA, Submap* submapB) {
  return true;
}

int SubmapStitching::findNeighboors(const Submap& s,
                                    std::vector<int>* neighboor_ids) {
  return 0;
}

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

SubmapStitching::c_info_t::c_info_t(int a0, Point a1, Point a2)
    : n(a0), point(a1), normal(a2) {}

}  // namespace panoptic_mapping
