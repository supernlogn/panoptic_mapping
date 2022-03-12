#ifndef PANOPTIC_MAPPING_MAP_MANAGEMENT_MAP_MANAGER_H_
#define PANOPTIC_MAPPING_MAP_MANAGEMENT_MAP_MANAGER_H_

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/map/pose_manager.h"
#include "panoptic_mapping/map/pseudo_submap.h"
#include "panoptic_mapping/map/submap.h"
#include "panoptic_mapping/map/submap_collection.h"
#include "panoptic_mapping/map_management/activity_manager.h"
#include "panoptic_mapping/map_management/layer_manipulator.h"
#include "panoptic_mapping/map_management/map_manager_base.h"
// TODO(supernlogn): This is an ugly fix for not getting
// redefinition warning. See if it can be done otherwise.
#define BOOST_PARAMETER_MAX_ARITY_PREV BOOST_PARAMETER_MAX_ARITY
#undef BOOST_PARAMETER_MAX_ARITY
#include "panoptic_mapping/map_management/plane_collection.h"
#undef BOOST_PARAMETER_MAX_ARITY
#define BOOST_PARAMETER_MAX_ARITY BOOST_PARAMETER_MAX_ARITY_PREV
#include "panoptic_mapping/map_management/tsdf_registrator.h"

#include "cblox_msgs/MapHeader.h"
#include "cblox_msgs/MapPoseUpdates.h"
#include "tf/tf.h"
#include "voxblox_msgs/Submap.h"

namespace panoptic_mapping {

/**
 * @brief High level class that wraps all map management actions and tools.
 */
class MapManager : public MapManagerBase {
 public:
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;

    // Perform actions every n ticks (frames), set 0 to turn off.
    int prune_active_blocks_frequency = 0;
    int change_detection_frequency = 0;
    int activity_management_frequency = 0;
    int update_poses_with_voxgraph_frequency = 0;
    int publish_poses_to_voxgraph_frequency = 0;
    // how many poses to put in one batch to be merged to one
    // and send them to voxgraph
    int num_submaps_to_merge_for_voxgraph = 1;
    // If true deactivated background submaps
    // don't get merged with other submaps.
    bool avoid_merging_deactivated_background_submaps = false;
    // If true background submaps will be published (and sent) to a voxgraph
    // node, when they get deactivated
    bool send_deactivated_submaps_to_voxgraph = false;
    // If true, submaps that are deactivated are checked for alignment with
    // inactive maps and merged together if a match is found.
    bool merge_deactivated_submaps_if_possible = false;

    // If true, the class layer will be integrated into the TSDF layer and
    // discraded afterwards when submaps are deactivated. This saves memory at
    // the loss of classification information.
    bool apply_class_layer_when_deactivating_submaps = false;
    // If true the submaps contained by a background submap
    // will also be updated using a weighted update
    bool update_contained_submaps_with_correction = false;
    // If true then the whole trajectory is updated when optimized
    // transformations are received from voxgraph
    bool update_whole_trajectory_with_voxgraph_tf = true;
    // If true(default) then submaps and poses get optimized, while
    // panoptic mapping is running. If false, optimizations are performed only
    // in finish mapping.
    bool optimize_poses_online = true;
    // publish to voxgraph topic names
    std::string background_submap_topic_name =
        "/panoptic_mapper/background_submap_out";
    std::string optimized_background_poses_topic_name =
        "/voxgraph_mapper/submap_poses";
    // Voxgraph's service to launch finish mapping
    // when panoptic mapping is about to exit
    std::string voxgraph_finish_map_srv_name = "/voxgraph_mapper/finish_map";
    // Voxgraph's service to save its stored trajectory
    std::string voxgraph_trajectory_srv_name =
        "/voxgraph_mapper/save_pose_history_to_file";
    // If not empty the trajectory stored in panoptic mapping is saved
    // to this file in a custom format
    std::string save_trajectory_on_finish = "";
    // If not empty the midposes are stored in a bag file of this
    // path.
    std::string debug_mid_poses_file = "";
    // If not empty then voxgraph's trajectory is saved to
    // this file as a .bag file
    std::string save_voxgraph_trajectory_on_finish = "";
    std::string input_odom_frame = "world";
    std::string robot_name = "robot";
    // If true then the planes of each submap are detected
    // and sent to voxgraph to stitch the submaps
    bool use_submap_stitching = true;
    // Member configs.
    TsdfRegistrator::Config tsdf_registrator_config;
    ActivityManager::Config activity_manager_config;
    LayerManipulator::Config layer_manipulator_config;
    PlaneCollection::Config plane_collection_config;
    Config() { setConfigName("MapManager"); }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  explicit MapManager(const Config& config);
  virtual ~MapManager() = default;

  // Perform all actions when with specified timings.
  void tick(SubmapCollection* submaps) override;
  void finishMapping(SubmapCollection* submaps) override;

  // Perform specific tasks.
  void pruneActiveBlocks(SubmapCollection* submaps);
  void manageSubmapActivity(SubmapCollection* submaps);
  void performChangeDetection(SubmapCollection* submaps);
  /**
   * @brief Updates the background submaps when new ones are received
   * from Voxgraph. It should run in each cycle to have as much updated
   * submaps as possible.
   *
   * @param submaps
   */
  void optimizePosesWithVoxgraphPoses(SubmapCollection* submaps);

  // Tools.
  bool mergeSubmapIfPossible(SubmapCollection* submaps, int submap_id,
                             int* merged_id = nullptr);
  /**
   * @brief Receives an optimized middle pose from Voxgraph
   *  for a previously published deactivated background submap.
   *  It computes the transformation that can transform the
   *  middle pose of this background submap in panoptic mapping
   *  to the optimized one received from Voxgraph.
   *
   * @param msg The message comming from voxgraph with
   *            the optimized pose for the last background submap
   */
  void optimizedVoxgraphPosesCallback(const cblox_msgs::MapPoseUpdates& msg);

 protected:
  /**
   * @brief Filters all newly deactivated background submaps and sents each
   * unpublished one to publishSubmapToVoxGraph. It should run in each cycle to
   * have as much updated submaps as possible.
   *
   * @param submaps
   */
  void updatePublishedSubmaps(SubmapCollection* submaps);

  /**
   * @brief Publishes a given submap to voxgraph's submap topic.
   * @param submapToPublish the most recent deactivated background submap to be
   * published to voxgraph The publishing is done in the topic defined by
   * optimized_background_poses_topic_name in config.
   */
  void publishSubmapToVoxGraph(SubmapCollection* submaps,
                               const Submap& submapToPublish);
  /**
   * @brief Merges a pseudo submap fo A into a pseudo submap B.
   *
   * @param submapA a PseudoSubmap to merge with PseudoSubmap B.
   *                This will be invalid after the merge.
   * @param submapB This is a PseudoSubmap which will take place
   *                into merging but also the merging result will be stored
   * here.
   */
  void mergePseudoSubmapAToPseudoSubmapB(const PseudoSubmap& submapA,
                                         PseudoSubmap* submapB);

  std::string pruneBlocks(Submap* submap) const;

 private:
  static config_utilities::Factory::RegistrationRos<MapManagerBase, MapManager>
      registration_;
  // Members.
  const Config config_;

  std::shared_ptr<ActivityManager> activity_manager_;
  std::shared_ptr<TsdfRegistrator> tsdf_registrator_;
  std::shared_ptr<LayerManipulator> layer_manipulator_;
  std::shared_ptr<PlaneCollection> plane_collection_;
  PoseManager* pose_manager_;
  // For publishing background to voxgraph
  ros::NodeHandle nh_;
  ros::Publisher background_submap_publisher_;
  std::vector<int> published_submap_ids_to_voxgraph_;
  std::vector<int> actualy_used_mid_pose_ids_;
  size_t sent_counter_;
  Transformation T_C_R_;
  // For receiving optimized poses from voxgraph
  ros::Subscriber optimized_background_poses_sub_;
  std::queue<cblox_msgs::MapPoseUpdates> voxgraph_correction_tfs_;
  size_t received_counter_;
  std::vector<PoseManager::poseIdType> pseudo_submaps_sent_mid_pose_ids_;
  // For voxgraph_correction_tfs_ access synchronization
  static std::mutex callback_mutex_;

  // Action tick counters.
  class Ticker {
   public:
    Ticker(unsigned int max_ticks,
           std::function<void(SubmapCollection* submaps)> action)
        : max_ticks_(max_ticks), action_(std::move(action)) {}
    void tick(SubmapCollection* submaps);

   private:
    unsigned int current_tick_ = 0;
    const unsigned int max_ticks_;
    const std::function<void(SubmapCollection* submaps)> action_;
  };
  std::vector<Ticker> tickers_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_MANAGEMENT_MAP_MANAGER_H_
