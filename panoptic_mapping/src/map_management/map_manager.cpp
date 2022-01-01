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
#include <std_srvs/Empty.h>
#include <voxblox_ros/conversions.h>
#include <voxgraph/frontend/submap_collection/voxgraph_submap.h>

namespace panoptic_mapping {

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
  setupParam("avoid_merging_deactivated_background_submaps",
             &avoid_merging_deactivated_background_submaps);
  setupParam("merge_deactivated_submaps_if_possible",
             &merge_deactivated_submaps_if_possible);
  setupParam("apply_class_layer_when_deactivating_submaps",
             &apply_class_layer_when_deactivating_submaps);
  setupParam("send_deactivated_submaps_to_voxgraph",
             &send_deactivated_submaps_to_voxgraph);
  setupParam("update_contained_submaps_with_voxblox",
             &update_contained_submaps_with_voxblox);
  setupParam("update_contained_submaps_sigma", &update_contained_submaps_sigma);
  setupParam("activity_manager_config", &activity_manager_config,
             "activity_manager");
  setupParam("tsdf_registrator_config", &tsdf_registrator_config,
             "tsdf_registrator");
  setupParam("layer_manipulator_config", &layer_manipulator_config,
             "layer_manipulator");
  setupParam("save_trajectory_on_finish", &save_trajectory_on_finish);
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
  if (config_.send_deactivated_submaps_to_voxgraph) {
    sent_counter_ = 0;
    received_counter_ = 0;
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
  // call voxgraph for one last time
  std_srvs::Empty empty_message;

  // Remove all empty blocks.
  std::stringstream info;
  info << "Finished mapping: ";
  for (Submap& submap : *submaps) {
    info << pruneBlocks(&submap);
  }
  LOG_IF(INFO, config_.verbosity >= 3) << info.str();

  // send the last background submaps to voxgraph
  // and call for a full graph optimization
  if (config_.send_deactivated_submaps_to_voxgraph) {
    publishSubmapToVoxGraph(submaps, *(submaps->getBackground()));
    while (sent_counter_ > received_counter_) {
      sleep(1);
    }
    LOG(INFO) << "Finished waiting the last voxgraph optimization";
    optimize_poses_from_voxgraph(submaps);
  }

  // Deactivate last submaps.
  for (Submap& submap : *submaps) {
    if (submap.isActive()) {
      LOG_IF(INFO, config_.verbosity >= 3)
          << "Deactivating submap " << submap.getID();
      submap.setBackground_id_on_deactivation(submaps->getBackgroundID());
      submap.finishActivePeriod();
    }
  }

  if (!config_.save_trajectory_on_finish.empty()) {
    LOG_IF(INFO, config_.verbosity >= 3)
        << "Saving trajectory to " << config_.save_trajectory_on_finish;
    std::ofstream fw(config_.save_trajectory_on_finish,
                     std::ios::out | std::ios::binary);
    // write trajectory to a file
    const auto trajectory = PoseManager::getGlobalInstance()->getAllPoses();
    fw << trajectory.size() << '\n';
    for (const auto pose : trajectory) {
      fw << pose.pose << '\n';
      fw << pose.pose_idx << '\n';
      fw << pose.time.toSec() << '\n';
    }
  }

  LOG_IF(INFO, config_.verbosity >= 3) << "Merging Submaps:";

  // Merge what is possible.
  bool merged_something = true;
  while (merged_something) {
    const int bckg_id = submaps->getBackgroundID();
    for (Submap& submap : *submaps) {
      if (!(submap.getID() == bckg_id &&
            config_.avoid_merging_deactivated_background_submaps)) {
        merged_something = mergeSubmapIfPossible(submaps, submap.getID());
        if (merged_something) {
          break;
        }
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
    submap->setBackground_id_on_deactivation(submaps->getBackgroundID());
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

    if (config_.avoid_merging_deactivated_background_submaps &&
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
          }
        }
        // transfer the submap_id appearing in one pose to the other id
        PoseManager::getGlobalInstance()->addSubmapIdToPoses(
            other.getID(), submap->getPoseHistory());
        PoseManager::getGlobalInstance()->removeSubmapIdFromPoses(
            submap_id, submap->getPoseHistory());
        submaps->removeSubmap(submap_id);
        LOG(INFO) << "removed submap " << submap_id;
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
    ++received_counter_;
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
        const Transformation T_optimal(tf_received.cast<FloatingPoint>());

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
        const Transformation mid_pose =
            PoseManager::getGlobalInstance()->getPoseTransformation(
                mid_pose_id);
        const auto log_mid_pose = Transformation::log(mid_pose);
        const double mid_pose_time = PoseManager::getGlobalInstance()
                                         ->getPoseInformation(mid_pose_id)
                                         .time.toSec();
        // get the submap and change its pose
        if (submaps->submapIdExists(*id_it)) {
          Submap* submapToChange = submaps->getSubmapPtr(*id_it);

          const Transformation T_M_S_correction =
              PoseManager::getGlobalInstance()->getPoseCorrectionTF(mid_pose_id,
                                                                    T_optimal);
          const Transformation& submaps_initial_pose =
              submapToChange->getT_M_Sinit();

          submapToChange->setT_M_S(submaps_initial_pose * T_M_S_correction);
          if (config_.update_contained_submaps_with_voxblox) {
            const auto correction_log_tf =
                Transformation::log(T_M_S_correction);
            const auto connected_submaps_ids =
                PoseManager::getGlobalInstance()->getConnectedSubmaps(*id_it);
            int submaps_changed = 0;
            for (const int c_s_id : connected_submaps_ids) {
              if (!submaps->submapIdExists(c_s_id)) {
                continue;
              }
              ++submaps_changed;
              Submap* c_s = submaps->getSubmapPtr(c_s_id);
              int r_b_id = c_s->getResponsibleBackground();
              if (r_b_id >= 0) {
                // get middle pose of the connected submap
                const Submap::PoseIdHistory& c_s_pose_id_hist =
                    c_s->getPoseHistory();
                auto c_s_poseHistory_iterator = c_s_pose_id_hist.begin();
                std::advance(c_s_poseHistory_iterator,
                             c_s_pose_id_hist.size() / 2);
                const int c_s_middle_pose_id = *c_s_poseHistory_iterator;
                // compute the update weight function
                // a gaussian type is used here
                const Transformation c_s_middle_pose =
                    PoseManager::getGlobalInstance()->getPoseTransformation(
                        c_s_middle_pose_id);
                const auto log_c_s_middle_pose =
                    Transformation::log(c_s_middle_pose);
                const double c_s_time =
                    PoseManager::getGlobalInstance()
                        ->getPoseInformation(c_s_middle_pose_id)
                        .time.toSec();
                const double weight_value = poseInterpolationFunction(
                    log_c_s_middle_pose, log_mid_pose, c_s_time, mid_pose_time);
                // do the update
                const Transformation& c_s_initial_pose = c_s->getT_M_Sinit();
                c_s->setT_M_S(
                    c_s_initial_pose *
                    Transformation::exp(correction_log_tf * weight_value));
              }
            }
            LOG_IF(INFO, config_.verbosity >= 4)
                << submaps_changed << "out of" << connected_submaps_ids.size()
                << "submaps related to background changed \n";
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
      pose_msg.header.stamp = ros::Time(pose_info.time);
      pose_msg.header.frame_id = "world";
      const Transformation& pose =
          PoseManager::getGlobalInstance()->getPoseTransformation(pose_id);
      tf::poseKindrToMsg(pose.cast<double>(), &pose_msg.pose);
      submap_msg.trajectory.poses.emplace_back(pose_msg);
    }
  }
  LOG_IF(INFO, config_.verbosity >= 4)
      << "Publishing submap to Voxgraph with "
      << new_pseudo_submap.getPoseHistory().size() << " poses" << std::endl;
  background_submap_publisher_.publish(submap_msg);
  ++sent_counter_;
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

double MapManager::poseInterpolationFunction(
    const Eigen::Matrix<voxblox::FloatingPoint, 6, 1>& point_log_pose,
    const Eigen::Matrix<voxblox::FloatingPoint, 6, 1>& base_log_pose,
    const double point_time, const double base_time) const {
  const double sigma_weighted_update = config_.update_contained_submaps_sigma;
  return exp(-(point_log_pose - base_log_pose).squaredNorm() /
             (2 * sigma_weighted_update));
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
