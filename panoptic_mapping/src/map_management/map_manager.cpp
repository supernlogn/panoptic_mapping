#include "panoptic_mapping/map_management/map_manager.h"

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <cblox_ros/submap_conversions.h>
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <voxblox_msgs/FilePath.h>
#include <voxblox_ros/conversions.h>
#include <voxgraph/frontend/submap_collection/voxgraph_submap.h>

namespace panoptic_mapping {

Transformation computeT_C_R(const double rotX, const double rotY,
                            const double rotZ) {
  Eigen::Matrix3f rotMat1, rotMat2, rotMat3;
  kindr::minimal::AngleAxis rot1(rotX * M_PI / 180.0, Eigen::Vector3d::UnitX());
  kindr::minimal::AngleAxis rot2(rotY * M_PI / 180.0, Eigen::Vector3d::UnitY());
  kindr::minimal::AngleAxis rot3(rotZ * M_PI / 180.0, Eigen::Vector3d::UnitZ());
  rotMat1 =
      rot1.getRotationMatrix()
          .cast<voxblox::FloatingPoint>();  // -90 around x-axis --> rotation
                                            // around the side axis2
  rotMat2 = rot2.getRotationMatrix().cast<voxblox::FloatingPoint>();
  rotMat3 = rot3.getRotationMatrix().cast<voxblox::FloatingPoint>();
  voxblox::Quaternion q123(rotMat1 * rotMat2 * rotMat3);
  const Transformation T_S_O(Eigen::Vector3f(0, 0, 0), q123);
  return T_S_O;
}

config_utilities::Factory::RegistrationRos<MapManagerBase, MapManager>
    MapManager::registration_("submaps");

std::mutex MapManager::callback_mutex_;

void MapManager::Config::checkParams() const {
  checkParamConfig(activity_manager_config);
  checkParamConfig(tsdf_registrator_config);
  checkParamConfig(layer_manipulator_config);
  checkParamEq(num_submaps_to_merge_for_voxgraph % 2, 1,
               "num_submaps_to_merge_for_voxgraph should always be odd");
}

void MapManager::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("prune_active_blocks_frequency", &prune_active_blocks_frequency);
  setupParam("activity_management_frequency", &activity_management_frequency);
  setupParam("change_detection_frequency", &change_detection_frequency);
  setupParam("update_poses_with_voxgraph_frequency",
             &update_poses_with_voxgraph_frequency);
  setupParam("num_submaps_to_merge_for_voxgraph",
             &num_submaps_to_merge_for_voxgraph);
  setupParam("avoid_merging_deactivated_backhround_submaps",
             &avoid_merging_deactivated_backhround_submaps);
  setupParam("merge_deactivated_submaps_if_possible",
             &merge_deactivated_submaps_if_possible);
  setupParam("apply_class_layer_when_deactivating_submaps",
             &apply_class_layer_when_deactivating_submaps);
  setupParam("send_deactivated_submaps_to_voxgraph",
             &send_deactivated_submaps_to_voxgraph);
  setupParam("activity_manager_config", &activity_manager_config,
             "activity_manager");
  setupParam("tsdf_registrator_config", &tsdf_registrator_config,
             "tsdf_registrator");
  setupParam("layer_manipulator_config", &layer_manipulator_config,
             "layer_manipulator");
  setupParam("background_submap_topic_name", &background_submap_topic_name);
  setupParam("optimized_background_poses_topic_name",
             &optimized_background_poses_topic_name);
  setupParam("save_trajectory_on_finish", &save_trajectory_on_finish);
  setupParam("save_voxgraph_trajectory_on_finish",
             &save_voxgraph_trajectory_on_finish);
  setupParam("voxgraph_trajectory_srv_name", &voxgraph_trajectory_srv_name);
  setupParam("voxgraph_finish_map_srv_name", &voxgraph_finish_map_srv_name);
  setupParam("update_whole_trajectory_with_voxgraph_tf",
             &update_whole_trajectory_with_voxgraph_tf);
}

MapManager::MapManager(const Config& config)
    : config_(config.checkValid()), nh_("") {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();

  // Setup members.
  activity_manager_ =
      std::make_shared<ActivityManager>(config_.activity_manager_config);
  tsdf_registrator_ =
      std::make_shared<TsdfRegistrator>(config_.tsdf_registrator_config);
  layer_manipulator_ =
      std::make_shared<LayerManipulator>(config_.layer_manipulator_config);

  // Add all requested tasks.
  if (config_.prune_active_blocks_frequency > 0) {
    tickers_.emplace_back(
        config_.prune_active_blocks_frequency,
        [this](SubmapCollection* submaps) { pruneActiveBlocks(submaps); });
  }
  if (config_.activity_management_frequency > 0) {
    tickers_.emplace_back(
        config_.activity_management_frequency,
        [this](SubmapCollection* submaps) { manageSubmapActivity(submaps); });
  }
  if (config_.change_detection_frequency > 0) {
    tickers_.emplace_back(
        config_.change_detection_frequency,
        [this](SubmapCollection* submaps) { performChangeDetection(submaps); });
  }
  T_C_R_ = computeT_C_R(90.0, 0.0, 90.0);
  if (config.send_deactivated_submaps_to_voxgraph) {
    background_submap_publisher_ = nh_.advertise<voxblox_msgs::Submap>(
        config_.background_submap_topic_name, 100);
    optimized_background_poses_sub_ =
        nh_.subscribe(config_.optimized_background_poses_topic_name, 10,
                      &MapManager::optimized_voxgraph_submaps_callback, this);
    tickers_.emplace_back(config_.update_poses_with_voxgraph_frequency,
                          [this](SubmapCollection* submaps) {
                            optimize_poses_from_voxgraph(submaps);
                          });
    tickers_.emplace_back(
        config_.publish_poses_to_voxgraph_frequency,
        [this](SubmapCollection* submaps) { updatePublishedSubmaps(submaps); });
  }
}

void MapManager::tick(SubmapCollection* submaps) {
  // Increment counts for all tickers, which execute the requested actions.
  for (Ticker& ticker : tickers_) {
    ticker.tick(submaps);
  }
}

void MapManager::updatePublishedSubmaps(SubmapCollection* submaps) {
  // get all deactivated submaps
  std::unordered_set<int> deactivated_submaps;
  for (Submap& submap : *submaps) {
    if (!submap.isActive()) {
      deactivated_submaps.insert(submap.getID());
    }
  }
  // send deactivated background submaps to voxgraph
  for (int id : deactivated_submaps) {
    Submap* submap = submaps->getSubmapPtr(id);
    if (submap->getLabel() == PanopticLabel::kBackground) {
      if (std::find(published_submap_ids_to_voxgraph_.begin(),
                    published_submap_ids_to_voxgraph_.end(),
                    id) == published_submap_ids_to_voxgraph_.end()) {
        publishSubmapToVoxGraph(submaps, *submap);
      }
    }
  }
}

void MapManager::pruneActiveBlocks(SubmapCollection* submaps) {
  // Process all active instance and background submaps.
  auto t1 = std::chrono::high_resolution_clock::now();
  Timer timer("map_management/prune_active_blocks");
  std::stringstream info;
  std::vector<int> submaps_to_remove;
  for (Submap& submap : *submaps) {
    if (submap.getLabel() == PanopticLabel::kFreeSpace || !submap.isActive()) {
      continue;
    }
    info << pruneBlocks(&submap);

    // If a submap does not contain data anymore it can be removed.
    if (submap.getTsdfLayer().getNumberOfAllocatedBlocks() == 0) {
      submaps_to_remove.emplace_back(submap.getID());
      if (config_.verbosity >= 4) {
        info << "Removed submap " << submap.getID() << "!";
      }
    }
  }

  // Remove submaps.
  for (int id : submaps_to_remove) {
    for (int published_id : published_submap_ids_to_voxgraph_) {
      if (published_id == id) {
        LOG(INFO) << "HAVE TO remove submap : " << id
                  << "from published_submap_ids_to_voxgraph_"
                  << " at " << __LINE__ << std::endl;
        break;
      }
    }
    PoseManager::getGlobalInstance()->removeSubmapIdFromAllPoses(id);
    submaps->removeSubmap(id);
  }
  auto t2 = std::chrono::high_resolution_clock::now();
  timer.Stop();
  LOG_IF(INFO, config_.verbosity >= 2)
      << "Pruned active blocks in "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
      << "ms." << info.str();
}

void MapManager::manageSubmapActivity(SubmapCollection* submaps) {
  CHECK_NOTNULL(submaps);
  std::unordered_set<int> active_submaps;
  if (config_.merge_deactivated_submaps_if_possible) {
    // Track de-activated submaps if requested.
    for (const Submap& submap : *submaps) {
      if (submap.isActive()) {
        active_submaps.insert(submap.getID());
      }
    }
  }

  // Perform activity management.
  activity_manager_->processSubmaps(submaps);

  // Process de-activated submaps if requested.
  if (config_.merge_deactivated_submaps_if_possible ||
      config_.apply_class_layer_when_deactivating_submaps ||
      config_.send_deactivated_submaps_to_voxgraph) {
    // get all deactivated submaps
    std::unordered_set<int> deactivated_submaps;
    for (Submap& submap : *submaps) {
      if (!submap.isActive() &&
          active_submaps.find(submap.getID()) != active_submaps.end()) {
        deactivated_submaps.insert(submap.getID());
      }
    }

    // Apply the class layer if requested.
    if (config_.apply_class_layer_when_deactivating_submaps) {
      for (int id : deactivated_submaps) {
        Submap* submap = submaps->getSubmapPtr(id);
        submap->applyClassLayer(*layer_manipulator_);
      }
    }
    // Try to merge the submaps.
    if (config_.merge_deactivated_submaps_if_possible) {
      for (int id : deactivated_submaps) {
        if (submaps->submapIdExists(id)) {
          const Submap* submap = submaps->getSubmapPtr(id);
          if (config_.send_deactivated_submaps_to_voxgraph &&
              submap->getLabel() == PanopticLabel::kBackground) {
            continue;
          }
        }
        int merged_id;
        int current_id = id;
        while (mergeSubmapIfPossible(submaps, current_id, &merged_id)) {
          current_id = merged_id;
        }
        if (current_id == id) {
          LOG_IF(INFO, config_.verbosity >= 4)
              << "Submap " << id << " was deactivated, could not be matched."
              << std::endl;
        }
      }
    }
  }
}

void MapManager::performChangeDetection(SubmapCollection* submaps) {
  tsdf_registrator_->checkSubmapCollectionForChange(submaps);
}

void MapManager::finishMapping(SubmapCollection* submaps) {
  // Remove all empty blocks.
  std::stringstream info;
  info << "Finished mapping: ";
  for (Submap& submap : *submaps) {
    info << pruneBlocks(&submap);
  }
  LOG_IF(INFO, config_.verbosity >= 3) << info.str();

  // Deactivate last submaps.
  for (Submap& submap : *submaps) {
    if (submap.isActive()) {
      LOG_IF(INFO, config_.verbosity >= 3)
          << "Deactivating submap " << submap.getID();
      submap.finishActivePeriod();
    }
  }
  std::string save_voxgraph_trajectory_on_finish =
      "/home/ioannis/datasets/voxgraph_traj.bag";
  LOG_IF(INFO, config_.verbosity >= 3)
      << "Saving Voxgraph trajectory to:" << save_voxgraph_trajectory_on_finish;
  ros::ServiceClient sec = nh_.serviceClient<voxblox_msgs::FilePath>(
      "/voxgraph_mapper/save_pose_history_to_file");
  voxblox_msgs::FilePath msg;
  msg.request.file_path = save_voxgraph_trajectory_on_finish;
  if (!sec.call(msg)) {
    LOG(ERROR) << "sec.call(msg) cannot be called. sec.exists()="
               << sec.exists();
  }
  LOG_IF(INFO, config_.verbosity >= 3) << "Merging Submaps:";

  // Merge what is possible.
  bool merged_something = true;
  while (merged_something) {
    for (Submap& submap : *submaps) {
      merged_something = mergeSubmapIfPossible(submaps, submap.getID());
      if (merged_something) {
        break;
      }
    }
  }

  // Finish submaps.
  if (config_.apply_class_layer_when_deactivating_submaps) {
    LOG_IF(INFO, config_.verbosity >= 3) << "Applying class layers:";
    std::vector<int> empty_submaps;
    for (Submap& submap : *submaps) {
      if (submap.hasClassLayer()) {
        if (!submap.applyClassLayer(*layer_manipulator_)) {
          empty_submaps.emplace_back(submap.getID());
        }
      }
    }
    for (const int id : empty_submaps) {
      for (int published_id : published_submap_ids_to_voxgraph_) {
        if (published_id == id) {
          LOG(INFO) << "HAVE TO remove submap : " << id
                    << "from published_submap_ids_to_voxgraph_"
                    << " at " << __LINE__ << std::endl;
          break;
        }
      }

      submaps->removeSubmap(id);

      LOG_IF(INFO, config_.verbosity >= 3)
          << "Removed submap " << id << " which was empty.";
    }
  }
}

bool MapManager::mergeSubmapIfPossible(SubmapCollection* submaps, int submap_id,
                                       int* merged_id) {
  // Use on inactive submaps, checks for possible matches with other inactive
  // submaps.
  if (!submaps->submapIdExists(submap_id)) {
    return false;
  }
  // Setup.
  Submap* submap = submaps->getSubmapPtr(submap_id);
  if (submap->isActive()) {
    // Active submaps need first to be de-activated.
    submap->finishActivePeriod();
  } else if (submap->getChangeState() == ChangeState::kAbsent) {
    return false;
  }

  // Find all potential matches.
  for (Submap& other : *submaps) {
    if (other.isActive() || other.getClassID() != submap->getClassID() ||
        other.getID() == submap->getID() ||
        !submap->getBoundingVolume().intersects(other.getBoundingVolume())) {
      continue;
    }

    if (config_.avoid_merging_deactivated_backhround_submaps &&
        !other.isActive() && other.getLabel() == PanopticLabel::kBackground) {
      continue;
    }

    bool submaps_match;
    if (!tsdf_registrator_->submapsConflict(*submap, other, &submaps_match)) {
      if (submaps_match) {
        // It's a match, merge the submap into the candidate.
        // Make sure both maps have or don't have class layers.
        if (!(submap->hasClassLayer() && other.hasClassLayer())) {
          submap->applyClassLayer(*layer_manipulator_);
          other.applyClassLayer(*layer_manipulator_);
        }
        layer_manipulator_->mergeSubmapAintoB(*submap, &other);
        LOG_IF(INFO, config_.verbosity >= 4)
            << "Merged Submap " << submap->getID() << " into " << other.getID()
            << ".";
        other.setChangeState(ChangeState::kPersistent);
        for (auto it = published_submap_ids_to_voxgraph_.begin();
             it != published_submap_ids_to_voxgraph_.end(); ++it) {
          if (*it == submap_id) {
            LOG(INFO) << "HAVE TO remove submap : " << submap_id
                      << "from published_submap_ids_to_voxgraph_"
                      << " at " << __LINE__ << std::endl;
            *it = other.getID();
            if (merged_id) {
              *merged_id = other.getID();
            }
          }
        }

        // transfer the submap_id appearing in one pose to the other id
        PoseManager::getGlobalInstance()->addSubmapIdToPoses(
            other.getID(), submap->getPoseHistory());
        PoseManager::getGlobalInstance()->removeSubmapIdFromPoses(
            submap_id, submap->getPoseHistory());
        submaps->removeSubmap(submap_id);
        ROS_INFO("removed submap %d", submap_id);
        if (merged_id) {
          *merged_id = other.getID();
        }
        return true;
      }
    }
  }
  return false;
}

void MapManager::optimized_voxgraph_submaps_callback(
    const cblox_msgs::MapPoseUpdates& msg) {
  LOG_IF(INFO, config_.verbosity >= 4)
      << "MapManager::optimized_voxgraph_submaps_callback"
      << "received for time " << msg.header.stamp.toSec() << "with"
      << msg.map_headers.size() << " map_headers."
      << "Current number of submap ids stored is: "
      << published_submap_ids_to_voxgraph_.size() << std::endl;
  // push the message received to the queue of messages.
  {
    std::lock_guard<std::mutex> guard(callback_mutex_);
    voxgraph_correction_tfs_.emplace(msg);
  }
}

void MapManager::optimize_poses_from_voxgraph(SubmapCollection* submaps) {
  // The elements of the voxgraph_correction_tfs_ queue are updated in
  // optimized_voxgraph_submaps_callback when a message with optimized poses
  // is received from Voxgraph. The front elements of the queue are used
  // here to update the background submap sent to voxgraph and which the
  // optimized pose regards to. To find this submap, the
  // published_submap_ids_to_voxgraph_ is used.

  // Do all pending corrections
  if (!published_submap_ids_to_voxgraph_.empty()) {
    std::lock_guard<std::mutex> guard(callback_mutex_);
    while (!voxgraph_correction_tfs_.empty()) {
      const cblox_msgs::MapPoseUpdates& msg = voxgraph_correction_tfs_.front();
      // if n submaps were merged into 1 pseudosubmap and sent
      // then the submaps ids are enumbered as  0,1,...,n-1
      // and we should correct the (n/2+1)-th --> 0 + n//2
      auto id_it = published_submap_ids_to_voxgraph_.begin() +
                   (config_.num_submaps_to_merge_for_voxgraph / 2);
      int loop_index = 0;
      // for all the map headers in the message
      for (const auto map_header : msg.map_headers) {
        // get the actual transformation coming from Voxgraph
        kindr::minimal::QuatTransformationTemplate<double> tf_received;
        tf::poseMsgToKindr(map_header.pose_estimate.map_pose, &tf_received);
        const Transformation T_M_S_optimal(tf_received.cast<FloatingPoint>());

        if (id_it == published_submap_ids_to_voxgraph_.end()) {
          ROS_ERROR("There are less ids than map_headers");
          break;
        }
        // get middle pose from pose history
        const Submap::PoseIdHistory& poseHistory =
            pseudo_submaps_sent_[loop_index].getPoseHistory();
        auto poseHistory_iterator = poseHistory.begin();
        std::advance(poseHistory_iterator, poseHistory.size() / 2);
        const int mid_pose_id = *poseHistory_iterator;
        // get the submap and change its pose
        if (submaps->submapIdExists(*id_it)) {
          Submap* submapToChange = submaps->getSubmapPtr(*id_it);

          const Transformation T_M_S_correction =
              PoseManager::getGlobalInstance()->getPoseCorrectionTF(
                  mid_pose_id, T_M_S_optimal, T_C_R_);
          submapToChange->setT_M_S(submapToChange->getT_M_Sinit() *
                                   T_M_S_correction);
          if (config_.update_whole_trajectory_with_voxgraph_tf) {
            PoseManager::getGlobalInstance()->correctSubmapPoses(
                *id_it, T_M_S_correction);
          }
          LOG_IF(INFO, config_.verbosity >= 4)
              << "pose for submap id:" << *id_it << " was set" << std::endl;
        } else {
          ROS_ERROR("TRYING TO CHANGE SUBMAP ID %d THAT DOES NOT EXIST",
                    *id_it);
        }
        ++id_it;
        ++loop_index;
      }
      voxgraph_correction_tfs_.pop();
    }
  }
}

void MapManager::mergePseudoSubmapAToPseudoSubmapB(const PseudoSubmap& submapA,
                                                   PseudoSubmap* submapB) {
  // actual merging per voxel
  layer_manipulator_->mergePseudoSubmapAintoB(submapA, submapB);
  LOG_IF(INFO, config_.verbosity >= 4) << "Merged PseudoSubmaps .";
  // merging of pose history
  // by putting all times to submapB in sorting order
  // assumes that poseHistory in submapB is already sorted per time
  const Submap::PoseIdHistory& mA = submapA.getPoseHistoryConst();
  Submap::PoseIdHistory& mB = submapB->getPoseHistory();
  mB.insert(mA.begin(), mA.end());
}

void MapManager::publishSubmapToVoxGraph(SubmapCollection* submaps,
                                         const Submap& submapToPublish) {
  // This code is from Voxblox::TsdfServer::publishSubmap
  // assume that submapToPublish is not null
  published_submap_ids_to_voxgraph_.emplace_back(submapToPublish.getID());
  if (published_submap_ids_to_voxgraph_.size() <
      config_.num_submaps_to_merge_for_voxgraph) {
    ROS_WARN("NOT ENOUGH submaps to publish");
    return;
  }
  // create a new merged submap by merging
  // all <published_submap_ids_to_voxgraph_> poses
  PseudoSubmap new_pseudo_submap(submapToPublish);
  int count = 1;  // starting from one, we already have one pseudo submap
  // for the <num_submaps_to_merge_for_voxgraph> most
  // recent deactivated background submaps
  for (auto it = published_submap_ids_to_voxgraph_.rbegin();
       it != published_submap_ids_to_voxgraph_.rend(); ++it) {
    if (count == config_.num_submaps_to_merge_for_voxgraph) {
      break;
    }
    ++count;
    int prev_id = *it;
    const Submap& prev_published_submap = submaps->getSubmap(prev_id);
    PseudoSubmap prev_pseudo_map(prev_published_submap);
    mergePseudoSubmapAToPseudoSubmapB(prev_pseudo_map, &new_pseudo_submap);
    LOG_IF(INFO, config_.verbosity >= 4)
        << "merging submaps" << submapToPublish.getID() << "and" << prev_id
        << " for publishing";
  }
  pseudo_submaps_sent_.emplace_back(new_pseudo_submap);
  // create the submap message
  voxblox_msgs::Submap submap_msg;
  {
    submap_msg.robot_name = "robot";
    voxblox::serializeLayerAsMsg<TsdfVoxel>(new_pseudo_submap.getTsdfLayer(),
                                            /* only_updated */ false,
                                            &submap_msg.layer);
    for (const auto pose_id : new_pseudo_submap.getPoseHistory()) {
      auto pose_info =
          PoseManager::getGlobalInstance()->getPoseInformation(pose_id);
      geometry_msgs::PoseStamped pose_msg;
      if (pose_info.time < ros::TIME_MIN) {
        pose_msg.header.stamp = ros::TIME_MIN;
      } else {
        pose_msg.header.stamp = pose_info.time;
      }
      pose_msg.header.frame_id = "world";
      const Transformation& pose =
          PoseManager::getGlobalInstance()->getPoseTransformation(pose_id) *
          T_C_R_;
      tf::poseKindrToMsg(pose.cast<double>(), &pose_msg.pose);
      submap_msg.trajectory.poses.emplace_back(pose_msg);
    }
  }
  LOG_IF(INFO, config_.verbosity >= 4)
      << "Publishing submap to Voxgraph with "
      << new_pseudo_submap.getPoseHistory().size() << " poses" << std::endl;
  background_submap_publisher_.publish(submap_msg);
}

std::string MapManager::pruneBlocks(Submap* submap) const {
  auto t1 = std::chrono::high_resolution_clock::now();
  // Setup.
  ClassLayer* class_layer = nullptr;
  if (submap->hasClassLayer()) {
    class_layer = submap->getClassLayerPtr().get();
  }
  TsdfLayer* tsdf_layer = submap->getTsdfLayerPtr().get();
  MeshLayer* mesh_layer = submap->getMeshLayerPtr().get();
  const int voxel_indices = std::pow(submap->getConfig().voxels_per_side, 3);
  int count = 0;

  // Remove all blocks that don't have any belonging voxels.
  voxblox::BlockIndexList block_indices;
  tsdf_layer->getAllAllocatedBlocks(&block_indices);
  for (const auto& block_index : block_indices) {
    const ClassBlock* class_block = nullptr;
    if (class_layer) {
      if (class_layer->hasBlock(block_index)) {
        class_block = &class_layer->getBlockByIndex(block_index);
      }
    }
    const TsdfBlock& tsdf_block = tsdf_layer->getBlockByIndex(block_index);
    bool has_beloning_voxels = false;

    // Check all voxels.
    for (int voxel_index = 0; voxel_index < voxel_indices; ++voxel_index) {
      if (tsdf_block.getVoxelByLinearIndex(voxel_index).weight >= 1e-6) {
        if (class_block) {
          if (classVoxelBelongsToSubmap(
                  class_block->getVoxelByLinearIndex(voxel_index))) {
            has_beloning_voxels = true;
            break;
          }
        } else {
          has_beloning_voxels = true;
          break;
        }
      }
    }

    // Prune blocks.
    if (!has_beloning_voxels) {
      if (class_layer) {
        class_layer->removeBlock(block_index);
      }
      tsdf_layer->removeBlock(block_index);
      mesh_layer->removeMesh(block_index);
      count++;
    }
  }
  auto t2 = std::chrono::high_resolution_clock::now();
  std::stringstream ss;
  if (count > 0 && config_.verbosity >= 4) {
    ss << "\nPruned " << count << " blocks from submap " << submap->getID()
       << " (" << submap->getName() << ") in "
       << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
       << "ms.";
  }
  return ss.str();
}

void MapManager::Ticker::tick(SubmapCollection* submaps) {
  // Perform 'action' every 'max_ticks' ticks.
  current_tick_++;
  if (current_tick_ >= max_ticks_) {
    action_(submaps);
    current_tick_ = 0;
  }
}

}  // namespace panoptic_mapping
