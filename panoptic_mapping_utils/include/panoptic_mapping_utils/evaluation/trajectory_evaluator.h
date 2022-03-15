#ifndef PANOPTIC_MAPPING_UTILS_EVALUATION_TRAJECTORY_EVALUATOR_H_
#define PANOPTIC_MAPPING_UTILS_EVALUATION_TRAJECTORY_EVALUATOR_H_

#include <string>
#include <utility>
#include <vector>

#include <panoptic_mapping/3rd_party/config_utilities.hpp>
#include <panoptic_mapping/3rd_party/nanoflann.hpp>
#include <panoptic_mapping/common/common.h>
#include <panoptic_mapping/map/pose_manager.h>
#include <ros/ros.h>

namespace panoptic_mapping {

class TrajectoryEvaluator {
 public:
  struct EvaluationRequest
      : public config_utilities::Config<EvaluationRequest> {
    int verbosity = 4;

    // Evaluation
    bool evaluate_only_optimized_points = false;
    std::string trajectory_file_path;
    std::string output_file_path;
    std::string generated_path_file_path;
    double max_distance_allowed = 100.0;
    EvaluationRequest() {
      setConfigName("TrajectoryEvaluator::EvaluationRequest");
    }

   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };

  // Constructor.
  TrajectoryEvaluator(const ros::NodeHandle& nh,
                      const ros::NodeHandle& nh_private);
  virtual ~TrajectoryEvaluator() = default;

  // Access.
  bool evaluate(const EvaluationRequest& request);

 private:
  // Members
  void readOptimizedTrajectory(const EvaluationRequest& request);
  void readGroundTruthTrajectory(
      const TrajectoryEvaluator::EvaluationRequest& request);
  void lcs(std::vector<std::pair<int, int> >* matched_pairs);
  // ROS.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Files.
  std::ofstream output_file_;

  // Stored data.
  std::vector<PoseManager::PoseInformation> ground_truth_trajectory_;
  std::vector<PoseManager::PoseInformation> drifted_trajectory_;
  std::vector<PoseManager::PoseInformation> optimized_trajectory_;

  EvaluationRequest request_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_UTILS_EVALUATION_TRAJECTORY_EVALUATOR_H_
