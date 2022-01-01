#include "panoptic_mapping_utils/evaluation/trajectory_evaluator.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <panoptic_mapping/3rd_party/config_utilities.hpp>
#include <ros/ros.h>

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

void read4x4MatrixFromFile(std::ifstream* ifile, Transformation* ret) {
  Transformation::TransformationMatrix mat;
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      double t;
      *ifile >> t;
      mat(i, j) = t;
    }
  }
  *ret = Transformation(mat);
}

void TrajectoryEvaluator::readOptimizedTrajectory(
    const TrajectoryEvaluator::EvaluationRequest& request) {
  // Load the provided trajectory.
  std::ifstream trajectory_file(request.trajectory_file_path,
                                std::ios::in | std::ios::binary);
  int num_trajectory_points = -1;
  if (!trajectory_file.is_open()) {
    LOG(ERROR) << "trajectory file at " << request.trajectory_file_path
               << " cannot be opened";
  }
  trajectory_file >> num_trajectory_points;
  LOG_IF(INFO, request.verbosity > 1)
      << "num_trajectory_points: " << num_trajectory_points << '\n';

  for (int i = 0; i < num_trajectory_points; ++i) {
    PoseManager::PoseInformation p_info;
    double t;
    read4x4MatrixFromFile(&trajectory_file, &p_info.pose);
    trajectory_file >> p_info.pose_idx;
    trajectory_file >> t;
    p_info.time = ros::Time(t);
    optimized_trajectory_.push_back(p_info);
  }
}

void TrajectoryEvaluator::readGroundTruthTrajectory(
    const TrajectoryEvaluator::EvaluationRequest& request) {
  // Load the groundtruth and noisy trajectories.
  std::ifstream gt_file(request.generated_path_file_path);
  int counter = 0;
  while (!gt_file.eof()) {
    PoseManager::PoseInformation p_info_gt;
    PoseManager::PoseInformation p_info_wd;
    std::string s;
    double time;
    char c;
    double rotation_x, rotation_y, rotation_z, rotation_w, translation_x,
        translation_y, translation_z;
    // reading time
    std::getline(gt_file, s, ':');
    gt_file >> time;
    // reading ground_truth pose
    // reading rotation
    std::getline(gt_file, s, '[');
    gt_file >> rotation_x;
    gt_file >> c >> rotation_y;
    gt_file >> c >> rotation_z;
    gt_file >> c >> rotation_w;
    // reading translation
    std::getline(gt_file, s, '[');
    gt_file >> translation_x;
    gt_file >> c >> translation_y;
    gt_file >> c >> translation_z;
    gt_file >> c;
    {
      Transformation::Rotation rotation(rotation_w, rotation_x, rotation_y,
                                        rotation_z);
      Transformation::Vector3 translation(translation_x, translation_y,
                                          translation_z);
      Transformation pose(translation, rotation);
      p_info_gt.pose = pose;
      p_info_gt.time = ros::Time(time);
    }
    ground_truth_trajectory_.push_back(p_info_gt);
    // reading pose with drift
    std::getline(gt_file, s, ':');
    // reading rotation
    std::getline(gt_file, s, '[');
    gt_file >> rotation_x;
    gt_file >> c >> rotation_y;
    gt_file >> c >> rotation_z;
    gt_file >> c >> rotation_w;
    // reading translation
    std::getline(gt_file, s, '[');
    gt_file >> translation_x;
    gt_file >> c >> translation_y;
    gt_file >> c >> translation_z;
    gt_file >> c;
    {
      Transformation::Rotation rotation(rotation_w, rotation_x, rotation_y,
                                        rotation_z);
      Transformation::Vector3 translation(translation_x, translation_y,
                                          translation_z);
      Transformation pose(translation, rotation);
      p_info_wd.pose = pose;
      p_info_wd.time = ros::Time(time);
    }
    drifted_trajectory_.push_back(p_info_wd);
    // reading till last character
    std::getline(gt_file, s, '\n');
  }
}

double distanceMetric(const Transformation& poseL,
                      const Transformation& poseR) {
  const auto logDiff = Transformation::log(poseL) - Transformation::log(poseR);
  return logDiff.squaredNorm();
}

double distanceMetricPInfo(const PoseManager::PoseInformation& p_infoL,
                           const PoseManager::PoseInformation& p_infoR) {
  return distanceMetric(p_infoL.pose, p_infoR.pose);
}

std::string computeTrajectoryDistances(
    const TrajectoryEvaluator::EvaluationRequest& request,
    const std::vector<PoseManager::PoseInformation>& trajectoryL,
    const std::vector<PoseManager::PoseInformation>& trajectoryR,
    const std::vector<std::pair<int, int> >& matched_pairs) {
  int numPoints = matched_pairs.size();
  double totalSqError = 0;
  double totalNormError = 0;
  int numIncludedPoints = 0;
  for (const auto mp : matched_pairs) {
    int iL = mp.first;
    int iR = mp.second;
    double r = distanceMetricPInfo(trajectoryL[iL], trajectoryR[iR]);
    if (r > request.max_distance_allowed) {
      continue;
    }
    numIncludedPoints++;
    totalSqError += r;
    totalNormError += sqrt(r);
  }
  std::stringstream ss;
  if (numPoints == 0) {
    ss << ","
       << ","
       << ",";
  } else {
    double mean = totalNormError / numPoints;
    double stddev = sqrt(totalSqError / (numPoints - 1));
    double rmse = sqrt(totalSqError / numPoints);
    ss << mean << "," << stddev << "," << rmse << "," << numIncludedPoints;
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
  output_file_ << "dg_mean[m],"
               << "dg_stddev[m],"
               << "dg_rmse[m],"
               << "dg_num_points,"
               << "og_mean[m],"
               << "og_stddev[m],"
               << "og_rmse[m],"
               << "og_num_points[m]" << '\n';
  output_file_ << dg_trajectories_errors << ',' << og_trajectories_errors
               << '\n';
  return true;
}

}  // namespace panoptic_mapping
