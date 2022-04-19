#include "panoptic_mapping_utils/evaluation/trajectory_evaluator.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/TransformStamped.h>
#include <minkindr_conversions/kindr_msg.h>
#include <panoptic_mapping/3rd_party/config_utilities.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "panoptic_mapping_utils/evaluation/progress_bar.h"

namespace panoptic_mapping {

void TrajectoryEvaluator::EvaluationRequest::checkParams() const {
  checkParamNE(trajectory_file_path.empty(), true, "trajectory_file_path");
  checkParamNE(output_file_path.empty(), true, "output_file_path");
  checkParamGT(max_distance_allowed, 0.0, "max_distance_allowed");
}

void TrajectoryEvaluator::EvaluationRequest::setupParamsAndPrinting() {
  setupParam("evaluate_only_optimized_points", &evaluate_only_optimized_points);
  setupParam("generated_path_file_path", &generated_path_file_path);
  setupParam("trajectory_file_path", &trajectory_file_path);
  setupParam("output_file_path", &output_file_path);
  setupParam("max_distance_allowed", &max_distance_allowed);
}

void TrajectoryEvaluator::readOptimizedTrajectory(
    const TrajectoryEvaluator::EvaluationRequest& request) {
  // Load the provided trajectory.
  {
    std::ifstream trajectory_file(request.trajectory_file_path,
                                  std::ios::in | std::ios::binary);
    if (!trajectory_file) {
      LOG(ERROR) << "trajectory file at " << request.trajectory_file_path
                 << " cannot be opened";
      return;
    }
  }
  const std::string topic = "pose_history";
  rosbag::Bag bag;
  bag.open(request.trajectory_file_path, rosbag::bagmode::Read);
  for (rosbag::MessageInstance const m :
       rosbag::View(bag, rosbag::TopicQuery(topic))) {
    PoseManager::PoseInformation p_info;
    geometry_msgs::PoseStamped::ConstPtr tfs =
        m.instantiate<geometry_msgs::PoseStamped>();
    if (tfs == nullptr) {
      continue;
    }
    kindr::minimal::QuatTransformationTemplate<double> pose_d;
    tf::poseMsgToKindr(tfs->pose, &pose_d);
    p_info.pose = pose_d.cast<float>();
    p_info.pose_init = pose_d.cast<float>();
    p_info.time = tfs->header.stamp;
    optimized_trajectory_.push_back(p_info);
  }
  bag.close();
}

void TrajectoryEvaluator::readGroundTruthTrajectory(
    const TrajectoryEvaluator::EvaluationRequest& request) {
  // Load the groundtruth and noisy trajectories.
  {
    std::ifstream gt_file(request.generated_path_file_path);
    if (!gt_file) {
      LOG(ERROR) << "file " << request.generated_path_file_path
                 << " does not exist";
      return;
    }
  }
  // std::string topic="pose_history"
  const std::string gt_topic = "data/pose_ground_truth";
  const std::string drifted_topic = "data/pose";
  rosbag::Bag bag;
  bag.open(request.generated_path_file_path, rosbag::bagmode::Read);
  for (rosbag::MessageInstance const m :
       rosbag::View(bag, rosbag::TopicQuery(gt_topic))) {
    PoseManager::PoseInformation p_info_gt;
    geometry_msgs::TransformStamped::ConstPtr tfs =
        m.instantiate<geometry_msgs::TransformStamped>();
    if (tfs == nullptr) {
      continue;
    }
    kindr::minimal::QuatTransformationTemplate<double> pose_d;
    tf::transformMsgToKindr(tfs->transform, &pose_d);
    p_info_gt.pose = pose_d.cast<float>();
    p_info_gt.pose_init = pose_d.cast<float>();
    p_info_gt.time = tfs->header.stamp;
    ground_truth_trajectory_.push_back(p_info_gt);
  }
  for (rosbag::MessageInstance const m :
       rosbag::View(bag, rosbag::TopicQuery(drifted_topic))) {
    PoseManager::PoseInformation p_info_gt;
    geometry_msgs::TransformStamped::ConstPtr tfs =
        m.instantiate<geometry_msgs::TransformStamped>();
    if (tfs == nullptr) {
      continue;
    }
    kindr::minimal::QuatTransformationTemplate<double> pose_d;
    tf::transformMsgToKindr(tfs->transform, &pose_d);
    p_info_gt.pose = pose_d.cast<float>();
    p_info_gt.pose_init = pose_d.cast<float>();
    p_info_gt.time = tfs->header.stamp;
    drifted_trajectory_.push_back(p_info_gt);
  }
  bag.close();
}

std::pair<double, double> distanceMetric(const Transformation& poseL,
                                         const Transformation& poseR) {
  double positional_error =
      (poseL.getPosition() - poseR.getPosition()).squaredNorm();
  const voxblox::Rotation qr =
      poseL.getRotation() * poseR.getRotation().inverse();
  double rotational_error =
      kindr::minimal::AngleAxisTemplate<float>(qr).angle();
  return std::make_pair(positional_error, rotational_error);
}

std::pair<double, double> distanceMetricPInfo(
    const PoseManager::PoseInformation& p_infoL,
    const PoseManager::PoseInformation& p_infoR) {
  return distanceMetric(p_infoL.pose, p_infoR.pose);
}

std::string computeTrajectoryDistances(
    const TrajectoryEvaluator::EvaluationRequest& request,
    const std::vector<PoseManager::PoseInformation>& trajectoryL,
    const std::vector<PoseManager::PoseInformation>& trajectoryR,
    const std::vector<std::pair<int, int> >& matched_pairs) {
  int numPoints = matched_pairs.size();
  double totalSqError[2] = {0, 0};
  double totalNormError[2] = {0, 0};
  int numIncludedPoints = 0;
  for (const auto mp : matched_pairs) {
    int iL = mp.first;
    int iR = mp.second;
    auto r = distanceMetricPInfo(trajectoryL[iL], trajectoryR[iR]);
    double r_pos = r.first;
    double r_rot = r.second;
    if (r_pos > request.max_distance_allowed) {
      continue;
    }
    numIncludedPoints++;
    totalSqError[0] += r_pos;
    totalSqError[1] += r_rot * r_rot;
    totalNormError[0] += sqrt(r_pos);
    totalNormError[1] += r_rot;
  }
  std::stringstream ss;
  if (numPoints == 0) {
    ss << ","
       << ","
       << ",";
  } else {
    double mean[2] = {totalNormError[0] / numPoints,
                      totalNormError[1] / numPoints};
    double stddev[2] = {sqrt(totalSqError[0] / (numPoints - 1)),
                        sqrt(totalSqError[1] / (numPoints - 1))};
    double rmse[2] = {sqrt(totalSqError[0] / numPoints),
                      sqrt(totalSqError[1] / numPoints)};
    ss << mean[0] << "," << mean[1] << "," << stddev[0] << "," << stddev[1]
       << "," << rmse[0] << "," << rmse[1] << "," << numIncludedPoints;
  }

  return ss.str();
}

void TrajectoryEvaluator::lcs(
    std::vector<std::pair<int, int> >* matched_pairs) {
  int i = 0;
  int j = 0;
  while (i < optimized_trajectory_.size() &&
         j < ground_truth_trajectory_.size()) {
    double og_sec = optimized_trajectory_[i].time.toSec();
    double gt_sec = ground_truth_trajectory_[i].time.toSec();
    if (optimized_trajectory_[i].time.toSec() ==
        ground_truth_trajectory_[j].time.toSec()) {
      matched_pairs->emplace_back(i, j);
      i++;
      j++;
    } else if (j + 1 < ground_truth_trajectory_.size()) {
      j++;
    }
  }
}

TrajectoryEvaluator::TrajectoryEvaluator(const ros::NodeHandle& nh,
                                         const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {}

bool TrajectoryEvaluator::evaluate(
    const TrajectoryEvaluator::EvaluationRequest& request) {
  if (!request.isValid(true)) {
    return false;
  }
  LOG_IF(INFO, request.verbosity >= 2) << "Processing: \n"
                                       << request.toString();

  // Load the groundtruth and drifted trajectories.
  readGroundTruthTrajectory(request);
  // Load the provided trajectory.
  readOptimizedTrajectory(request);

  int diff_index =
      ground_truth_trajectory_.size() - optimized_trajectory_.size();
  std::vector<std::pair<int, int> > matched_pairs;
  // LCS of sorted arrays
  lcs(&matched_pairs);
  // assert that paths have the same size
  LOG(INFO) << "ground_truth_trajectory_.size():"
            << ground_truth_trajectory_.size() << " "
            << "optimized_trajectory_.size():" << optimized_trajectory_.size();

  if (request.evaluate_only_optimized_points) {
    std::vector<int> mathed_pairs_to_remove;
    for (int i = 0; i < matched_pairs.size(); ++i) {
      const auto& mp = matched_pairs[i];
      const int iL = mp.first;
      const int iR = mp.second;
      if (optimized_trajectory_[iL].time.toSec() ==
          drifted_trajectory_[iR].time.toSec()) {
        mathed_pairs_to_remove.push_back(i);
      }
    }
    LOG_IF(INFO, request.verbosity > 3)
        << "removing " << mathed_pairs_to_remove.size() << " pairs";
    for (auto it = mathed_pairs_to_remove.rbegin();
         it != mathed_pairs_to_remove.rend(); it++) {
      matched_pairs.erase(matched_pairs.begin() + *it);
    }
  }
  // compute distance of the paths
  // compute distance between the drifted trajectory and groundtruth
  std::string dg_trajectories_errors;
  dg_trajectories_errors = computeTrajectoryDistances(
      request, drifted_trajectory_, ground_truth_trajectory_, matched_pairs);
  // compute distance between the optimized trajectory and groundtruth
  std::string og_trajectories_errors;
  og_trajectories_errors = computeTrajectoryDistances(
      request, optimized_trajectory_, ground_truth_trajectory_, matched_pairs);
  output_file_.open(request.output_file_path);
  output_file_ << "dg_mean_pos[m],"
               << "dg_mean_rot[rad],"
               << "dg_stddev_pos[m],"
               << "dg_stddev_rot[rad],"
               << "dg_rmse_pos[m],"
               << "dg_rmse_rot[rad],"
               << "dg_num_points,"
               << "og_mean_pos[m],"
               << "og_mean_rot[rad],"
               << "og_stddev_pos[m],"
               << "og_stddev_rot[rad],"
               << "og_rmse_pos[m],"
               << "og_rmse_rot[rad],"
               << "og_num_points[m]" << '\n';
  output_file_ << dg_trajectories_errors << ',' << og_trajectories_errors
               << '\n';
  return true;
}

}  // namespace panoptic_mapping
