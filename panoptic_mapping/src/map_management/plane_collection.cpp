#include "panoptic_mapping/map_management/plane_collection.h"

#include <algorithm>
#include <cmath>
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

template <typename T>
std::vector<size_t> argsort(const std::vector<T>& array) {
  std::vector<size_t> indices(array.size());
  std::iota(indices.begin(), indices.end(), 0);
  std::sort(indices.begin(), indices.end(),
            [&array](int left, int right) -> bool {
              // sort indices according to corresponding array element
              return array[left] > array[right];
            });

  return indices;
}

int PlaneCollection::seed_num_ = 2;
std::mt19937 PlaneCollection::random_number_generator_;

void PlaneCollection::Config::checkParams() const {
  checkParamGT(z_threshold, 0.0f, "z_threshold");
  checkParamGT(xy_threshold, 0.0f, "xy_threshold");
  checkParamGT(ransac_num_iterations, 0, "ransac_num_iterations");
  checkParamGT(ransac_probability, 0.0, "ransac_probability");
  const int minPoints = ransac_min_points;
  checkParamGT(minPoints, 0, "ransac_min_points");
  checkParamGT(ransac_epsilon, 0.0, "ransac_epsilon");
  checkParamGT(ransac_cluster_epsilon, 0.0, "ransac_cluster_epsilon");
  checkParamGE(ransac_normal_threshold, 0.0, "ransac_normal_threshold");
  checkParamCond(class_planes_merge_to_class.size() == classes.size(),
                 "class_planes_merge_to_class wrong size");
  checkParamCond(classes_max_instances.size() == classes.size(),
                 "classes_max_instances wrong size");
}

void PlaneCollection::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("z_threshold", &z_threshold);
  setupParam("xy_threshold", &xy_threshold);
  setupParam("classes", &classes);
  setupParam("classes_max_instances", &classes_max_instances);
  setupParam("class_planes_merge_to_class", &class_planes_merge_to_class);
  if (class_planes_merge_to_class.empty()) {
    class_planes_merge_to_class = classes;
  }
  setupParam("ransac_num_iterations", &ransac_num_iterations);
  setupParam("ransac_probability", &ransac_probability);
  int min_points = static_cast<int>(ransac_min_points);
  setupParam("ransac_min_points", &min_points);
  ransac_min_points = static_cast<size_t>(min_points);
  setupParam("ransac_epsilon", &ransac_epsilon);
  setupParam("ransac_cluster_epsilon", &ransac_cluster_epsilon);
  setupParam("ransac_normal_threshold", &ransac_normal_threshold);
  setupParam("max_outlier_percentage", &max_outlier_percentage);
  setupParam("satisfying_outlier_percent", &satisfying_outlier_percent);
  setupParam("plane_visualization_topic", &plane_visualization_topic);
}

PlaneCollection::PlaneCollection(const Config& config)
    : config_(config.checkValid()) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
  set_seed(config_.random_generator_seed);
  submap_id_to_class_to_planes_ =
      std::make_shared<std::map<int, classToPlanesType>>(
          std::map<int, classToPlanesType>());
  cgalSetPlaneExtractor();
  if (!config_.plane_visualization_topic.empty()) {
    visual_tools_visualizer_ =
        std::make_shared<rviz_visual_tools::RvizVisualTools>(
            "world", config_.plane_visualization_topic);
  }
  CHECK_NE(config_.plane_visualization_topic.empty(), true);
  CHECK_NE(visual_tools_visualizer_.get(), nullptr);
}

void PlaneCollection::processSubmap(Submap* s) {
  std::map<PlaneCollection::ClassID, std::vector<PointIndexType>>
      filtered_class_indices;
  CHECK(s->hasClassLayer());
  // if (s->getIsoSurfacePoints().size() == 0u) {
  //   s->updateEverything();
  // }
  if (s->getIsoSurfacePoints().size() == 0u) {
    return;
  }
  // CHECK_NE(s->getIsoSurfacePoints().size(), 0u);
  LOG(INFO) << "Calling plane collection";
  applyClassPreFilter(&filtered_class_indices, s->getTsdfLayer(),
                      s->getMeshLayer(), s->getClassLayer());
  // s->getIsoSurfacePoints();
  classToPlanesType class_id_to_planes;
  Transformation T_mid_pose =
      PoseManager::getGlobalInstance()->getPoseTransformation(
          s->getMidPoseID());
  findSubmapPlanes(&class_id_to_planes, s->getMeshLayer(), T_mid_pose,
                   filtered_class_indices);
  std::stringstream ss;
  ss << "class -- num_points -- num_planes";
  for (const auto& p : class_id_to_planes) {
    ss << "\n"
       << p.first << "--" << filtered_class_indices.at(p.first).size() << "--"
       << p.second.size();
    int dst_class = config_.class_planes_merge_to_class[p.first];
    if (p.first != dst_class) {
      for (const auto plane : class_id_to_planes.at(p.first)) {
        class_id_to_planes.at(dst_class).emplace_back(plane);
      }
      class_id_to_planes[p.first].clear();
    }
  }
  LOG(INFO) << "\n" << ss.str();
  submap_id_to_class_to_planes_->insert({s->getID(), class_id_to_planes});
  visualizePlanesOfClasses(class_id_to_planes);
}

void PlaneCollection::findSubmapPlanes(
    classToPlanesType* result, const voxblox::MeshLayer& mesh_layer,
    const Transformation T_mid_pose,
    const std::map<PlaneCollection::ClassID, std::vector<PointIndexType>>&
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
      cgalExtractPlane(&result->at(class_id), mesh_layer, T_mid_pose,
                       p_indices_pair.second, config_.ransac_num_iterations,
                       max_num_planes, class_id);
    } else {
      LOG_IF(INFO, config_.verbosity >= 4) << "skipping class with no indices";
    }
  }
}

bool PlaneCollection::cgalExtractPlane(
    std::vector<PlaneType>* merged_result, const voxblox::MeshLayer& mesh_layer,
    const Transformation& T_mid_pose,
    const std::vector<PointIndexType>& p_indices, const int num_iterations,
    const int max_num_planes, const int class_id) {
  size_t num_points = p_indices.size();
  if (num_points < config_.ransac_min_points) {
    return false;
  }
  Pwn_vector mesh_points(num_points);
  std::vector<const Point*> eigen_mesh_points(num_points);
  std::vector<const Point*> eigen_mesh_normals(num_points);
  {
    int i = 0;
    for (const auto& p_idx : p_indices) {
      const auto& p_n = getPointAndNormalFromPointIndex(p_idx, mesh_layer);
      eigen_mesh_points[i] = p_n.first;
      eigen_mesh_normals[i] = p_n.second;
      mesh_points[i].first =
          Kernel::Point_3(p_n.first->x(), p_n.first->y(), p_n.first->z());
      mesh_points[i].second =
          Kernel::Vector_3(p_n.second->x(), p_n.second->y(), p_n.second->z());
      ++i;
    }
    LOG_IF(INFO, config_.verbosity >= 3)
        << "Planeransac: Initialized mesh_points pointers";
  }
  Efficient_ransac shape_detector;
  // Registers planar shapes via template method.
  shape_detector.template add_shape_factory<Cgal_plane_type>();
  // Provides the input data.
  shape_detector.set_input(mesh_points);
  shape_detection_parameters_.min_points =
      std::max(num_points / static_cast<size_t>(max_num_planes),
               std::min(config_.ransac_min_points, num_points));
  // Detects registered shapes with default parameters.
  shape_detector.detect(shape_detection_parameters_);

  LOG_IF(INFO, config_.verbosity >= 4)
      << shape_detector.shapes().end() - shape_detector.shapes().begin()
      << " shapes detected.";
  // find out which planes have the most points
  std::vector<size_t> num_points_per_plane;
  for (const auto& cshape : shape_detector.shapes()) {
    num_points_per_plane.push_back(cshape->indices_of_assigned_points().size());
  }
  // keep only the planes with the most points
  std::vector<size_t> indices = argsort(num_points_per_plane);
  size_t count = 0;
  const size_t num_new_planes = shape_detector.shapes().size();
  for (const auto idx : indices) {
    if (count >= max_num_planes) {
      break;
    }
    const Efficient_ransac::Shape* cshape =
        (shape_detector.shapes().begin() + idx)->get();
    const Cgal_plane_type* cplane =
        dynamic_cast<const Cgal_plane_type*>(cshape);
    float d = static_cast<float>(cplane->d());
    Eigen::Vector3d eigen_normal(cplane->plane_normal().x(),
                                 cplane->plane_normal().y(),
                                 cplane->plane_normal().z());
    eigen_normal.stableNormalize();
    Point p = eigen_normal.cast<float>() * static_cast<float>(d);
    LOG_IF(INFO, config_.verbosity >= 5) << "eigen_normal:\n" << eigen_normal;
    // if it is wall like (see small flat class ids) then add it to walls
    int class_id_final = config_.class_planes_merge_to_class[class_id];
    merged_result->emplace_back(eigen_normal.cast<float>().stableNormalized(),
                                p, class_id_final);
    PlaneType* full_plane = &merged_result->back();
    std::vector<const Point*> temp_points;
    std::vector<const Point*> temp_normals;
    for (const auto& idx : cplane->indices_of_assigned_points()) {
      temp_points.push_back(
          eigen_mesh_points[idx]);  // if this works with [] and not with .at it
                                    // is very suspicious
      temp_normals.push_back(eigen_mesh_normals[idx]);
    }
    full_plane->createPlaneSegmentAaBb(temp_points,
                                       config_.position_cluster_threshold);
    Point v = T_mid_pose.getPosition() - full_plane->getPointInit();
    // full_plane->fixNormal(temp_points, temp_normals);
    if (v.dot(full_plane->getPlaneNormal()) < 0) {
      full_plane->reverseNormal();
    }
    ++count;
    if (full_plane->getNumPoints() < 1000) {
      merged_result->pop_back();
    }
  }
  return true;
}

void PlaneCollection::cgalSetPlaneExtractor() {
  // Sets parameters for shape detection.
  shape_detection_parameters_.probability = config_.ransac_probability;
  shape_detection_parameters_.epsilon = config_.ransac_epsilon;
  shape_detection_parameters_.cluster_epsilon = config_.ransac_cluster_epsilon;
  shape_detection_parameters_.normal_threshold =
      config_.ransac_normal_threshold;
}

int PlaneCollection::ransacCheck(
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

int PlaneCollection::ransacCheckSingle(
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

Eigen::Hyperplane<float, 3> PlaneCollection::createPlaneFrom3Points(
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

void PlaneCollection::applyClassPreFilter(
    std::map<PlaneCollection::ClassID, std::vector<PointIndexType>>* ret,
    const TsdfLayer& tsdf_layer, const voxblox::MeshLayer& mesh_layer,
    const ClassLayer& class_layer) {
  CHECK_NOTNULL(ret);
  LOG_IF(INFO, config_.verbosity >= 3)
      << "applying class pre-filter for mesh layer with "
      << mesh_layer.getNumberOfAllocatedMeshes()
      << " allocated meshes and class layer with "
      << class_layer.getNumberOfAllocatedBlocks() << " number of blocks";
  // assign all background class IDs a zero-element vector
  LOG_IF(INFO, config_.verbosity >= 3)
      << "Initializing prefilter for background classes";
  const auto background_class_ids = config_.classes;
  for (ClassID class_id : background_class_ids) {
    ret->insert({class_id, std::vector<PointIndexType>()});
  }
  voxblox::BlockIndexList mesh_indices;
  mesh_layer.getAllAllocatedMeshes(
      &mesh_indices);  // TODO(supernlogn): See if this is not needed
  // iterate over all submap blocks
  size_t counter = 0;
  // Create an interpolator to interpolate the vertex weights from the TSDF.
  voxblox::Interpolator<TsdfVoxel> interpolator(&tsdf_layer);

  for (const voxblox::BlockIndex& block_index : mesh_indices) {
    voxblox::Mesh::ConstPtr mesh = mesh_layer.getMeshPtrByIndex(block_index);
    panoptic_mapping::ClassBlock::ConstPtr class_block =
        class_layer.getBlockConstPtrByIndex(block_index);
    int voxel_type = static_cast<int>(class_block->getVoxelType());
    // iterate over all indices of this block
    const size_t num_vertices = mesh->vertices.size();
    for (size_t i = 0u; i < num_vertices; ++i) {
      const auto& vertex = mesh->vertices[i];
      const auto& voxel_ptr = class_block->getVoxelByCoordinates(vertex);
      TsdfVoxel voxel;
      bool res = interpolator.getVoxel(vertex, &voxel, true);
      if (res) {
        const auto p_w = voxel.weight;
        if (voxel_ptr.isObserverd() && p_w > 0.9f) {
          ClassID class_id = voxel_ptr.getBelongingID();
          // add this point index to its class list
          if (ret->find(class_id) != ret->end()) {
            ret->at(class_id).emplace_back(block_index, i);
            // } else {
            //   ret->insert({class_id,
            //   std::vector<PointIndexType>{{block_index, i}}});
          }
        }
      }
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

// publish planes with normals
void PlaneCollection::visualizePlanesOfClasses(
    const classToPlanesType& class_to_planes) {
  if (config_.plane_visualization_topic.empty()) {
    return;
  }
  for (const auto& pair : class_to_planes) {
    visualizePlanesOfClass(pair.first, pair.second);
  }
  visual_tools_visualizer_->trigger();
}

void PlaneCollection::visualizePlanesOfClass(
    const ClassID class_id, const std::vector<PlaneType>& planes) {
  static int marker_id = 0;
  CHECK(visual_tools_visualizer_);
  for (const auto& plane : planes) {
    rviz_visual_tools::colors plane_color;
    if (class_id == 2) {
      plane_color = rviz_visual_tools::RED;
    } else if (class_id == 1) {
      plane_color = rviz_visual_tools::GREEN;
    } else {
      plane_color = rviz_visual_tools::BLUE;
    }
    plane.publishPlaneVisualization(visual_tools_visualizer_, plane_color,
                                    marker_id++);
  }
}

// general
int PlaneCollection::getMaxNumPlanesPerType(ClassID class_id) const {
  // TODO(supernlogn): define them arbitary
  const auto it =
      std::find(config_.classes.begin(), config_.classes.end(), class_id);
  if (it != config_.classes.end()) {
    int dist = it - config_.classes.begin();
    int ret = config_.classes_max_instances[dist];
    return ret;
  }
  LOG(WARNING) << "Not using such class " << class_id << " for plane matching";
  return 0;
}

std::pair<const Point*, const Point*>
PlaneCollection::getPointAndNormalFromPointIndex(
    const PointIndexType& p_idx, const voxblox::MeshLayer& mesh_layer) const {
  const auto& block = mesh_layer.getMeshPtrByIndex(p_idx.block_index);
  return std::make_pair(&block->vertices[p_idx.linear_index],
                        &block->normals[p_idx.linear_index]);
}

const Point& PlaneCollection::getNormalFromPointIndex(
    const PointIndexType& p_idx, const voxblox::MeshLayer& mesh_layer) const {
  return mesh_layer.getMeshPtrByIndex(p_idx.block_index)
      ->normals[p_idx.linear_index];
}

const Point& PlaneCollection::getPointFromPointIndex(
    const PointIndexType& p_idx, const voxblox::MeshLayer& mesh_layer) const {
  return mesh_layer.getMeshPtrByIndex(p_idx.block_index)
      ->vertices[p_idx.linear_index];
}

PlaneCollection::c_info_t::c_info_t(int a0, Point a1, Point a2)
    : n(a0), point(a1), normal(a2) {}

}  // namespace panoptic_mapping
