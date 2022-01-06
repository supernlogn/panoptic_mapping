#include "panoptic_mapping/map_management/interpolation_function.h"

#include <algorithm>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

namespace panoptic_mapping {

template <class T, std::size_t N>
constexpr std::size_t size(const T (&array)[N]) noexcept {
  return N;
}

void Interpolator::Config::checkParams() const {
  static std::string func_names[6] = {"none",
                                      "linear",
                                      "linearTime",
                                      "exponentialSimple",
                                      "exponentialSimpleTime",
                                      "exponentialKernel"};
  static_assert(6 == FunctionType::exponentialKernel + 1);
  checkParamNE(
      std::find(std::begin(func_names), std::end(func_names), func_name),
      std::end(func_names), func_name);
}

void Interpolator::Config::setupParamsAndPrinting() {
  setupParam("verbosity", &verbosity);
  setupParam("func_name", &func_name);
  setupParam("function_config", &function_config);
}

Interpolator::FunctionType Interpolator::FunctionTypeFromString(
    const std::string& s) {
  static std::string func_names[6] = {"none",
                                      "linear",
                                      "linearTime",
                                      "exponentialSimple",
                                      "exponentialSimpleTime",
                                      "exponentialKernel"};
  static_assert(6 == FunctionType::exponentialKernel + 1);
  const auto it = std::find(std::begin(func_names), std::end(func_names), s);
  if (it == std::end(func_names)) {
    return FunctionType::none;
  } else {
    return (FunctionType)std::distance(std::begin(func_names), it);
  }
}

std::string Interpolator::FunctionTypeToString(const FunctionType ft) {
  std::string ret = "";
  switch (ft) {
    case none:
      ret = "none";
      break;
    case linear:
      ret = "linear";
      break;
    case linearTime:
      ret = "linearTime";
      break;
    case exponentialSimple:
      ret = "exponentialSimple";
      break;
    case exponentialSimpleTime:
      ret = "exponentialSimpleTime";
      break;
    case exponentialKernel:
      ret = "exponentialKernel";
      break;
  }
  return ret;
}

std::unique_ptr<InterpolatorFunctionBase>
Interpolator::FunctionTypeToInterpolatorFunction(
    const FunctionType ft, const std::vector<double>& function_config) {
  std::unique_ptr<InterpolatorFunctionBase> ret = nullptr;
  switch (ft) {
    case none:
      ret = std::make_unique<InterpolatorFunctionNone>(function_config);
      break;
    case linear:
      ret = std::make_unique<InterpolatorFunctionLinear>();
      break;
    case linearTime:
      ret = std::make_unique<InterpolatorFunctionLinearTime>();
      break;
    case exponentialSimple:
      ret = std::make_unique<InterpolatorFunctionExponentialSimple>(
          function_config[0], function_config[1]);
      break;
    case exponentialSimpleTime:
      ret = std::make_unique<InterpolatorFunctionExponentialSimpleTime>(
          function_config[0], function_config[1]);
      break;
      // case exponentialKernel:
      //   ret = std::make_unique<InterpolatorFunctionExponentialKernel>();
      //   break;
  }
  return ret;
}

Interpolator::Interpolator(const Config& config, const ros::NodeHandle& nh)
    : config_(config.checkValid()), nh_("") {
  LOG_IF(INFO, config_.verbosity >= 1) << "\n" << config_.toString();
  FunctionType ft = FunctionTypeFromString(config_.func_name);
  interpolator_ =
      FunctionTypeToInterpolatorFunction(ft, config_.function_config);
}

InterpolationResult Interpolator::interpolate(const V6_t& point_log_pose,
                                              const V6_t& start_log_pose,
                                              const V6_t& end_log_pose,
                                              const double point_time,
                                              const double start_time,
                                              const double end_time) const {
  InterpolationResult ret;
  if (start_time == -1) {
    ret = interpolator_->interpolateSingle(point_log_pose, end_log_pose,
                                           point_time, end_time);
  } else {
    ret = interpolator_->interpolateDouble(point_log_pose, start_log_pose,
                                           end_log_pose, point_time, start_time,
                                           end_time);
  }
  return ret;
}

InterpolationResult InterpolatorFunctionLinear::interpolateSingle(
    const V6_t& point_log_pose, const V6_t& mid_log_pose,
    const double point_time, const double mid_time) const {
  double dist = (point_log_pose - mid_log_pose).norm();
  if (dist <= trivial_dist) {
    return {0.0, 1.0};
  }
  V6_t intepolated_point;

  return {0.0, (1.0 - dist / max_dist)};
}

InterpolationResult InterpolatorFunctionLinear::interpolateDouble(
    const V6_t& point_log_pose, const V6_t& start_log_pose,
    const V6_t& end_log_pose, const double point_time, const double start_time,
    const double end_time) const {
  // Inverse Distance Weighting (IDW) interpolation
  double dist_1 = (point_log_pose - start_log_pose).norm();
  double dist_2 = (end_log_pose - point_log_pose).norm();
  if (dist_1 <= trivial_dist) {
    return {1.0, 0.0};
  } else if (dist_2 <= trivial_dist) {
    return {0.0, 1.0};
  }
  double total_dist = dist_1 + dist_2;

  double weight_1 = dist_2 / total_dist;
  double weight_2 = dist_1 / total_dist;

  return {weight_1, weight_2};
}

InterpolationResult InterpolatorFunctionLinearTime::interpolateSingle(
    const V6_t& point_log_pose, const V6_t& mid_log_pose,
    const double point_time, const double mid_time) const {
  double dist = (point_log_pose - mid_log_pose).norm();
  if (dist <= trivial_dtime) {
    return {0.0, 1.0};
  }

  return {0.0, (1.0 - dist / max_dtime)};
}

InterpolationResult InterpolatorFunctionLinearTime::interpolateDouble(
    const V6_t& point_log_pose, const V6_t& start_log_pose,
    const V6_t& end_log_pose, const double point_time, const double start_time,
    const double end_time) const {
  double dist_1 = point_time - start_time;
  double dist_2 = end_time - point_time;
  if (dist_1 <= trivial_dtime) {
    return {1.0, 0.0};
  } else if (dist_2 <= trivial_dtime) {
    return {0.0, 1.0};
  }
  double total_dist = end_time - start_time;

  double weight_1 = dist_2 / total_dist;
  double weight_2 = dist_1 / total_dist;

  return {weight_1, weight_2};
}

InterpolationResult InterpolatorFunctionExponentialSimple::interpolateSingle(
    const V6_t& point_log_pose, const V6_t& mid_log_pose,
    const double point_time, const double mid_time) const {
  double weight =
      exp(-(point_log_pose - mid_log_pose).squaredNorm() / (2 * sigma_));
  return {0.0, weight};
}

InterpolationResult InterpolatorFunctionExponentialSimple::interpolateDouble(
    const V6_t& point_log_pose, const V6_t& start_log_pose,
    const V6_t& end_log_pose, const double point_time, const double start_time,
    const double end_time) const {
  double dist_1 = (point_log_pose - start_log_pose).squaredNorm();
  double dist_2 = (end_log_pose - point_log_pose).squaredNorm();

  double weight_1 = exp(-dist_1 / (2 * sigma_));
  double weight_2 = exp(-dist_2 / (2 * sigma_));
  double total_weight = 1.0;  // not gonna normalize

  return {weight_1 / total_weight, weight_2 / total_weight};
}

InterpolationResult
InterpolatorFunctionExponentialSimpleTime::interpolateSingle(
    const V6_t& point_log_pose, const V6_t& mid_log_pose,
    const double point_time, const double mid_time) const {
  double dtime = (point_time - mid_time);
  double weight = exp(-dtime * dtime / (2 * sigma_));
}

InterpolationResult
InterpolatorFunctionExponentialSimpleTime::interpolateDouble(
    const V6_t& point_log_pose, const V6_t& start_log_pose,
    const V6_t& end_log_pose, const double point_time, const double start_time,
    const double end_time) const {
  double dtime_1 = (point_time - start_time);
  double dtime_2 = (end_time - point_time);

  double weight_1 = exp(-dtime_1 * dtime_1 / (2 * sigma_));
  double weight_2 = exp(-dtime_2 * dtime_2 / (2 * sigma_));
  double total_weight = 1.0;  // not gonna normalize

  return {weight_1 / total_weight, weight_2 / total_weight};
}

// V6_t
// InterpolatorFunctionExponentialKernel::interpolateSingle(
//     const V6_t& point_log_pose,
//     const V6_t& mid_log_pose, const double point_time,
//     const double mid_time) const {}
// V6_t
// InterpolatorFunctionExponentialKernel::interpolateDouble(
//     const V6_t& point_log_pose,
//     const V6_t& start_log_pose,
//     const V6_t& end_log_pose, const double point_time,
//     const double start_time, const double end_time) const {}

}  // namespace panoptic_mapping
