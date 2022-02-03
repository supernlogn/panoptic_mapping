#include "panoptic_mapping/tracking/projective_id_tracker.h"

#include <future>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "panoptic_mapping/common/index_getter.h"

namespace panoptic_mapping {

config_utilities::Factory::RegistrationRos<IDTrackerBase, ProjectiveIDTracker,
                                           std::shared_ptr<Globals>>
    ProjectiveIDTracker::registration_("projective");

void ProjectiveIDTracker::Config::checkParams() const {
  checkParamGT(rendering_threads, 0, "rendering_threads");
  checkParamNE(depth_tolerance, 0.f, "depth_tolerance");
  checkParamGT(rendering_subsampling, 0, "rendering_subsampling");
  checkParamConfig(renderer);
}

void ProjectiveIDTracker::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("depth_tolerance", &depth_tolerance);
  setupParam("tracking_metric", &tracking_metric);
  setupParam("match_acceptance_threshold", &match_acceptance_threshold);
  setupParam("use_class_data_for_matching", &use_class_data_for_matching);
  setupParam("use_approximate_rendering", &use_approximate_rendering);
  setupParam("rendering_subsampling", &rendering_subsampling);
  setupParam("min_allocation_size", &min_allocation_size);
  setupParam("rendering_threads", &rendering_threads);
  setupParam("renderer", &renderer);
}

ProjectiveIDTracker::ProjectiveIDTracker(const Config& config,
                                         std::shared_ptr<Globals> globals,
                                         bool print_config)
    : IDTrackerBase(std::move(globals)),
      config_(config.checkValid()),
      renderer_(config.renderer, globals_->camera()->getConfig(), false) {
  LOG_IF(INFO, config_.verbosity >= 1 && print_config) << "\n"
                                                       << config_.toString();
  addRequiredInputs({InputData::InputType::kColorImage,
                     InputData::InputType::kDepthImage,
                     InputData::InputType::kSegmentationImage,
                     InputData::InputType::kValidityImage});
}

void ProjectiveIDTracker::processInput(SubmapCollection* submaps,
                                       InputData* input) {
  CHECK_NOTNULL(submaps);
  CHECK_NOTNULL(input);
  CHECK(inputIsValid(*input));
  // Visualization.
  cv::Mat input_vis;
  std::unique_ptr<Timer> vis_timer;
  if (visualizationIsOn()) {
    vis_timer = std::make_unique<Timer>("visualization/tracking");
    Timer timer("visualization/tracking/input_image");
    input_vis = renderer_.colorIdImage(input->idImage());
    vis_timer->Pause();
  }

  // Render all submaps.
  Timer timer("tracking");
  auto t0 = std::chrono::high_resolution_clock::now();
  Timer detail_timer("tracking/compute_tracking_data");
  TrackingInfoAggregator tracking_data = computeTrackingData(submaps, input);
  const auto back_fore_ids = seperateBackgroundIds(tracking_data.getInputIDs());
  // Assign the input ids to tracks or allocate new maps.
  detail_timer = Timer("tracking/match_ids");
  std::unordered_map<int, int> input_to_output;
  std::stringstream info;
  int n_matched = 0;
  int n_new = 0;
  Timer alloc_timer("tracking/allocate_submaps");
  alloc_timer.Pause();

  if (!submaps->backgroundExists()) {
    ++n_new;
  }
  processBackgroundInput(submaps, input, &input_to_output,
                         back_fore_ids.second);
  // Add current pose to all tracked submaps
  const auto pose_id = PoseManager::getGlobalInstance()->createPose(
      input->T_M_C(), ros::Time(input->timestamp()));

  for (const int input_id : back_fore_ids.first) {
    int submap_id;
    bool matched = false;
    float value;
    bool any_overlap;
    std::stringstream logging_details;
    // see if it is background and there is already an active background
    LabelEntry label;
    const bool label_exists = getLabelIfExists(input_id, &label);

    if (config_.use_class_data_for_matching || config_.verbosity >= 4) {
      std::vector<std::pair<int, float>> ids_values;
      any_overlap = tracking_data.getAllMetrics(input_id, &ids_values,
                                                config_.tracking_metric);
      // Check for classes if requested.
      if (config_.use_class_data_for_matching) {
        for (const auto& id_value : ids_values) {
          // These are ordered in decreasing overlap metric.
          if (id_value.second < config_.match_acceptance_threshold) {
            // No more matches possible.
            break;
          } else if (classesMatch(
                         input_id,
                         submaps->getSubmap(id_value.first).getClassID())) {
            // Found the best match.
            matched = true;
            submap_id = id_value.first;
            value = id_value.second;
            break;
          }
        }
      } else if (any_overlap && ids_values.front().second >
                                    config_.match_acceptance_threshold) {
        // Check only for the highest match.
        matched = true;
        submap_id = ids_values.front().first;
        value = ids_values.front().second;
      }

      // Print the matching statistics for all submaps.
      if (config_.verbosity >= 4) {
        logging_details << ". Overlap: ";
        for (const auto& id_value : ids_values) {
          logging_details << " " << id_value.first << "(" << std::fixed
                          << std::setprecision(2) << id_value.second << ")";
        }
      } else {
        logging_details << ". No overlap found.";
      }
    } else if (tracking_data.getHighestMetric(input_id, &submap_id, &value,
                                              config_.tracking_metric)) {
      // Only consider the highest metric candidate.
      if (value > config_.match_acceptance_threshold) {
        matched = true;
      }
    }

    // Allocate new submap if necessary and store tracking info.
    alloc_timer.Unpause();
    bool allocate_new_submap = tracking_data.getNumberOfInputPixels(input_id) >=
                               config_.min_allocation_size;
    if (matched) {
      n_matched++;
      input_to_output[input_id] = submap_id;
      submaps->getSubmapPtr(submap_id)->setWasTracked(true);
    } else if (allocate_new_submap) {
      n_new++;
      Submap* new_submap = allocateSubmap(input_id, submaps, input);
      if (new_submap) {
        input_to_output[input_id] = new_submap->getID();
      } else {
        input_to_output[input_id] = -1;
      }
    } else {
      // Ignore these.
      input_to_output[input_id] = -1;
    }
    alloc_timer.Pause();

    for (auto it = submaps->begin(); it != submaps->end(); it++) {
      if (it->isActive()) {
        PoseManager::getGlobalInstance()->addSubmapIdToPose(pose_id,
                                                            it->getID());
        it->addPoseID(pose_id);
      }
    }
    // Logging.
    if (config_.verbosity >= 3) {
      if (matched) {
        info << "\n  " << input_id << "->" << submap_id << " (" << std::fixed
             << std::setprecision(2) << value << ")";
      } else {
        if (allocate_new_submap) {
          info << "\n  " << input_id << "->" << input_to_output[input_id]
               << " [new]";
        } else {
          info << "\n  " << input_id << " [ignored]";
        }
        if (any_overlap) {
          info << " (" << std::fixed << std::setprecision(2) << value << ")";
        }
      }
      info << logging_details.str();
    }
  }
  detail_timer.Stop();
  int count_active_backgrounds = 0;
  for (auto it_submap = submaps->begin(); it_submap != submaps->end();
       it_submap++) {
    if (it_submap->getLabel() == PanopticLabel::kBackground &&
        it_submap->isActive()) {
      ++count_active_backgrounds;
    }
  }
  if (count_active_backgrounds > 1) {
    ROS_ERROR("in project_id_tracker active backgrounds: %d",
              count_active_backgrounds);
  }
  // Translate the id image.
  for (auto it = input->idImagePtr()->begin<int>();
       it != input->idImagePtr()->end<int>(); ++it) {
    *it = input_to_output[*it];
  }

  // Allocate free space map if required.
  alloc_timer.Unpause();
  freespace_allocator_->allocateSubmap(submaps, input);
  alloc_timer.Stop();

  // Finish.
  auto t1 = std::chrono::high_resolution_clock::now();
  timer.Stop();
  if (config_.verbosity >= 2) {
    LOG(INFO) << "Tracked IDs in "
              << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0)
                     .count()
              << "ms, " << n_matched << " matched, " << n_new
              << " newly allocated." << info.str();
  }

  // Publish Visualization if requested.
  if (visualizationIsOn()) {
    vis_timer->Unpause();
    Timer timer("visualization/tracking/rendered");
    if (config_.use_approximate_rendering) {
      rendered_vis_ = renderer_.colorIdImage(
          renderer_.renderActiveSubmapIDs(*submaps, input->T_M_C()));
    }
    timer = Timer("visualization/tracking/tracked");
    cv::Mat tracked_vis = renderer_.colorIdImage(input->idImage());
    timer.Stop();
    visualize(input_vis, "input");
    visualize(rendered_vis_, "rendered");
    visualize(input->colorImage(), "color");
    visualize(tracked_vis, "tracked");
    vis_timer->Stop();
  }
}

std::pair<std::vector<int>, std::vector<int>>
ProjectiveIDTracker::seperateBackgroundIds(const std::vector<int> input_ids) {
  std::vector<int> foreground_ids;
  std::vector<int> background_ids;
  for (const int input_id : input_ids) {  // TODO(supernlogn): check if stl
                                          // provides filter with two actions
    LabelEntry label;
    const bool label_exists = getLabelIfExists(input_id, &label);
    if (label_exists && label.label == PanopticLabel::kBackground) {
      background_ids.push_back(input_id);
    } else {
      foreground_ids.push_back(input_id);
    }
  }

  return std::make_pair(foreground_ids, background_ids);
}

void ProjectiveIDTracker::processBackgroundInput(
    SubmapCollection* submaps, InputData* input,
    std::unordered_map<int, int>* input_to_output,
    const std::vector<int> bck_input_ids) {
  // see if there is already an active background
  if (!submaps->backgroundExists()) {
    // create new background
    const int input_id = bck_input_ids[0];
    Submap* new_submap = allocateSubmap(input_id, submaps, input);
    new_submap->setLabel(PanopticLabel::kBackground);
    // this should occur only once, when there is no background submap
    submaps->setBackgroundID(new_submap->getID());
    new_submap->setIsActive(true);
  }
  Submap* bck_submap = submaps->getBackground();
  bck_submap->setWasTracked(true);
  // if background is just created
  // then the previous background needs to be provided
  // getMeshNormals(bck_submap);

  // get mean y (exact)
  const Transformation T_C_S =
      input->T_M_C().inverse() * bck_submap->getT_M_S();
  std::vector<size_t> limits(4);  // x_min, x_max, y_min, y_max
  const Camera::Config& cam_config = globals_->camera()->getConfig();
  limits = {0u, static_cast<size_t>(cam_config.width), 0u,
            static_cast<size_t>(cam_config.height)};
  const Transformation T_S_C = T_C_S.inverse();
  const TsdfLayer& tsdf_layer = bck_submap->getTsdfLayer();
  double mean_z = 0.0;
  size_t total_floor_points = 0u;
  bool floor_found = false;
  for (size_t u = limits[0]; u < limits[1];
       u += config_.rendering_subsampling) {
    for (size_t v = limits[2]; v < limits[3];
         v += config_.rendering_subsampling) {
      const float depth = input->depthImage().at<float>(v, u);
      if (depth < cam_config.min_range || depth > cam_config.max_range) {
        continue;
      }
      // TODO(supernlogn): Currently only for floor
      const int floor_label = 1;
      int input_id = input->idImage().at<int>(v, u);
      if (input_id != floor_label) {
        continue;
      }
      floor_found = true;
      const cv::Vec3f& vertex = input->vertexMap().at<cv::Vec3f>(v, u);
      const Point P_S = T_S_C * Point(vertex[0], vertex[1], vertex[2]);
      // const voxblox::BlockIndex block_index =
      //     tsdf_layer.computeBlockIndexFromCoordinates(P_S);
      // const auto block = tsdf_layer.getBlockPtrByIndex(block_index);
      mean_z += P_S[2];
      ++total_floor_points;
    }
  }
  // float diff_y = 0.0;
  // if (floor_found) {
  //   mean_z /= (double)total_floor_points;
  //   const float y_threshold = 0.1;
  //   if (has_floor_z_) {
  //     diff_y = current_floor_z - (float)mean_z;
  //     if (abs(diff_y) > y_threshold) {
  //       LOG(ERROR) << "y_threshold (" << diff_y << ") violated";
  //     } else {
  //       LOG(INFO) << "diff_y at:" << diff_y;
  //     }
  //   } else {
  //     has_floor_z_ = true;
  //     current_floor_z = (float)mean_z;
  //   }
  // } else {
  //   LOG(WARNING) << "No floor found: ";
  // }

  // Transformation current_T_M_C = input->T_M_C();
  // voxblox::Point pos = current_T_M_C.getPosition();
  // pos =  pos + voxblox::Point(0,0,diff_y);
  // Transformation new_T_M_C(current_T_M_C.getRotation(), pos);
  // input->setT_M_C(new_T_M_C);

  const int background_submap_id = bck_submap->getID();
  for (int bck_id : bck_input_ids) {
    input_to_output->insert({bck_id, background_submap_id});
  }
}

bool ProjectiveIDTracker::getLabelIfExists(const int input_id,
                                           LabelEntry* label) const {
  bool ret = globals_->labelHandler()->segmentationIdExists(input_id);
  if (ret) {
    *label = globals_->labelHandler()->getLabelEntry(input_id);
  }
  return ret;
}

Submap* ProjectiveIDTracker::allocateSubmap(int input_id,
                                            SubmapCollection* submaps,
                                            InputData* input) {
  LabelEntry label;
  if (globals_->labelHandler()->segmentationIdExists(input_id)) {
    label = globals_->labelHandler()->getLabelEntry(input_id);
  }
  return submap_allocator_->allocateSubmap(submaps, input, input_id, label);
}

bool ProjectiveIDTracker::classesMatch(int input_id, int submap_class_id) {
  if (!globals_->labelHandler()->segmentationIdExists(input_id)) {
    // Unknown ID.
    return false;
  }
  return globals_->labelHandler()->getClassID(input_id) == submap_class_id;
}

TrackingInfoAggregator ProjectiveIDTracker::computeTrackingData(
    SubmapCollection* submaps, InputData* input) {
  // Render each active submap in parallel to collect overlap statistics.
  SubmapIndexGetter index_getter(
      globals_->camera()->findVisibleSubmapIDs(*submaps, input->T_M_C()));
  std::vector<std::future<std::vector<TrackingInfo>>> threads(
      config_.rendering_threads);
  TrackingInfoAggregator tracking_data;
  for (int i = 0; i < config_.rendering_threads; ++i) {
    threads[i] = (std::async(
        std::launch::async,
        [this, i, &tracking_data, &index_getter, submaps,
         input]() -> std::vector<TrackingInfo> {
          // Also process the input image.
          if (i == 0) {
            tracking_data.insertInputImage(
                input->idImage(), input->depthImage(),
                globals_->camera()->getConfig(), config_.rendering_subsampling);
          }
          std::vector<TrackingInfo> result;
          int index;
          while (index_getter.getNextIndex(&index)) {
            if (config_.use_approximate_rendering) {
              result.emplace_back(this->renderTrackingInfoApproximate(
                  submaps->getSubmap(index), *input));
            } else {
              result.emplace_back(this->renderTrackingInfoVertices(
                  submaps->getSubmap(index), *input));
            }
          }
          return result;
        }));
  }

  // Join all threads.
  std::vector<TrackingInfo> infos;
  for (auto& thread : threads) {
    for (const TrackingInfo& info : thread.get()) {
      infos.emplace_back(std::move(info));
    }
  }
  tracking_data.insertTrackingInfos(infos);

  // Render the data if required.
  if (visualizationIsOn() && !config_.use_approximate_rendering) {
    Timer timer("visualization/tracking/rendered");
    cv::Mat vis =
        cv::Mat::ones(globals_->camera()->getConfig().height,
                      globals_->camera()->getConfig().width, CV_32SC1) *
        -1;
    for (const TrackingInfo& info : infos) {
      for (const Eigen::Vector2i& point : info.getPoints()) {
        vis.at<int>(point.y(), point.x()) = info.getSubmapID();
      }
    }
    rendered_vis_ = renderer_.colorIdImage(vis);
  }
  return tracking_data;
}

TrackingInfo ProjectiveIDTracker::renderTrackingInfoApproximate(
    const Submap& submap, const InputData& input) const {
  // Approximate rendering by projecting the surface points of the submap into
  // the camera and fill in a patch of the size a voxel has (since there is 1
  // vertex per voxel).

  // Setup.
  const Camera& camera = *globals_->camera();
  TrackingInfo result(submap.getID(), camera.getConfig());
  const Transformation T_C_S = input.T_M_C().inverse() * submap.getT_M_S();
  const float size_factor_x =
      camera.getConfig().fx * submap.getTsdfLayer().voxel_size() / 2.f;
  const float size_factor_y =
      camera.getConfig().fy * submap.getTsdfLayer().voxel_size() / 2.f;
  const float block_size = submap.getTsdfLayer().block_size();
  const FloatingPoint block_diag_half = std::sqrt(3.0f) * block_size / 2.0f;
  const float depth_tolerance =
      config_.depth_tolerance > 0
          ? config_.depth_tolerance
          : -config_.depth_tolerance * submap.getTsdfLayer().voxel_size();
  const cv::Mat& depth_image = input.depthImage();

  // Parse all blocks.
  voxblox::BlockIndexList index_list;
  submap.getMeshLayer().getAllAllocatedMeshes(&index_list);
  for (const voxblox::BlockIndex& index : index_list) {
    if (!camera.blockIsInViewFrustum(submap, index, T_C_S, block_size,
                                     block_diag_half)) {
      continue;
    }
    for (const Point& vertex :
         submap.getMeshLayer().getMeshByIndex(index).vertices) {
      // Project vertex and check depth value.
      const Point p_C = T_C_S * vertex;
      int u, v;
      if (!camera.projectPointToImagePlane(p_C, &u, &v)) {
        continue;
      }
      if (std::abs(depth_image.at<float>(v, u) - p_C.z()) >= depth_tolerance) {
        continue;
      }

      // Compensate for vertex sparsity.
      const int size_x = std::ceil(size_factor_x / p_C.z());
      const int size_y = std::ceil(size_factor_y / p_C.z());
      result.insertRenderedPoint(u, v, size_x, size_y);
    }
  }
  result.evaluate(input.idImage(), depth_image);
  return result;
}

TrackingInfo ProjectiveIDTracker::renderTrackingInfoVertices(
    const Submap& submap, const InputData& input) const {
  TrackingInfo result(submap.getID());

  // Compute the maximum extent to lookup vertices.
  const Transformation T_C_S = input.T_M_C().inverse() * submap.getT_M_S();
  const Point origin_C = T_C_S * submap.getBoundingVolume().getCenter();
  std::vector<size_t> limits(4);  // x_min, x_max, y_min, y_max
  const Camera::Config& cam_config = globals_->camera()->getConfig();

  // NOTE(schmluk): Currently just iterate over the whole frame since the sphere
  // tangent computation was not robustly implemented.
  size_t subsampling_factor = 1;
  limits = {0u, static_cast<size_t>(cam_config.width), 0u,
            static_cast<size_t>(cam_config.height)};
  const Transformation T_S_C = T_C_S.inverse();
  const TsdfLayer& tsdf_layer = submap.getTsdfLayer();
  const float depth_tolerance =
      config_.depth_tolerance > 0
          ? config_.depth_tolerance
          : -config_.depth_tolerance * submap.getTsdfLayer().voxel_size();
  for (size_t u = limits[0]; u < limits[1];
       u += config_.rendering_subsampling) {
    for (size_t v = limits[2]; v < limits[3];
         v += config_.rendering_subsampling) {
      const float depth = input.depthImage().at<float>(v, u);
      if (depth < cam_config.min_range || depth > cam_config.max_range) {
        continue;
      }
      const cv::Vec3f& vertex = input.vertexMap().at<cv::Vec3f>(v, u);
      const Point P_S = T_S_C * Point(vertex[0], vertex[1], vertex[2]);
      const voxblox::BlockIndex block_index =
          tsdf_layer.computeBlockIndexFromCoordinates(P_S);
      const auto block = tsdf_layer.getBlockPtrByIndex(block_index);
      if (block) {
        const size_t voxel_index =
            block->computeLinearIndexFromCoordinates(P_S);
        const TsdfVoxel& voxel = block->getVoxelByLinearIndex(voxel_index);
        bool classes_match = true;
        if (submap.hasClassLayer()) {
          const ClassVoxel& class_voxel =
              submap.getClassLayer()
                  .getBlockByIndex(block_index)
                  .getVoxelByLinearIndex(voxel_index);
          classes_match = classVoxelBelongsToSubmap(class_voxel);
        }
        if (voxel.weight > 1e-6 && std::abs(voxel.distance) < depth_tolerance) {
          result.insertVertexPoint(input.idImage().at<int>(v, u));
          if (visualizationIsOn()) {
            result.insertVertexVisualizationPoint(u, v);
          }
        }
      }
    }
  }
  return result;
}

}  // namespace panoptic_mapping
