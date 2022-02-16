#include "panoptic_mapping/map_management/submap_stitching.h"

#include <map>
#include <utility>
#include <vector>

#include "panoptic_mapping/common/index_getter.h"

namespace panoptic_mapping {

void SubmapStitching::Config::checkParams() const {
  checkParamGT(z_threshold, 0.0, "z_threshold");
  checkParamGT(xy_threshold, 0.0, "xy_threshold");
  checkParamGT(max_walls, 0, "max_walls");
  checkParamGT(max_floors, 0, "max_floors");
  checkParamGT(max_ceilings, 0, "max_ceilings");
}

void SubmapStitching::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("z_threshold", &z_threshold);
  setupParam("xy_threshold", &xy_threshold);
  setupParam("max_walls", &max_walls);
  setupParam("max_floors", &max_floors);
  setupParam("max_ceilings", &max_ceilings);
}

SubmapStitching::SubmapStitching(const Config& config)
    : config_(config.checkValid()) {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
}

void SubmapStitching::processSubmap(const Submap& s) { findSubmapPlanes(s); }

void SubmapStitching::findSubmapPlanes(const Submap& s) {}

bool SubmapStitching::stitch(const Submap& SubmapA, Submap* submapB) {}

int SubmapStitching::findNeighboors(const Submap& s,
                                    std::vector<int>* neighboor_ids) {}

bool SubmapStitching::planeRansac(
    std::map<ClassID, std::vector<Eigen::Hyperplane<float, 3> > >*
        merged_result,
    const int num_iterations) {}

std::vector<Eigen::Hyperplane<float, 3> > SubmapStitching::ransacSample(
    const std::vector<PointIndexType>& p_indices, const ClassID class_id) {}

std::map<SubmapStitching::ClassID, std::vector<PointIndexType> >
SubmapStitching::applyClassPreFilter() {}

Eigen::Hyperplane<float, 3> SubmapStitching::createPlaneFrom3Points(
    const Point& p1, const Point& p2, const Point& p3) const {}

std::pair<int, BoundingBoxType> SubmapStitching::ransac_check(
    const std::vector<Eigen::Hyperplane<float, 3> >& hyperplanes,
    const std::vector<PointIndexType>& p_indices) {}

// for stitching submap
void SubmapStitching::matchNeighboorPlanes(
    const Submap& SubmapA, const Submap& SubmapB,
    const std::vector<int>& neighboor_ids) {}

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
