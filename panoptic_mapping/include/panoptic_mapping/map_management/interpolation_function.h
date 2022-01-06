#ifndef PANOPTIC_MAPPING_MAP_MANAGEMENT_INTERPOLATION_FUNCTION_H_
#define PANOPTIC_MAPPING_MAP_MANAGEMENT_INTERPOLATION_FUNCTION_H_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "panoptic_mapping/3rd_party/config_utilities.hpp"
#include "panoptic_mapping/common/common.h"

#include "tf/tf.h"

namespace panoptic_mapping {

typedef Transformation::Vector6 V6_t;
class InterpolatorFunctionBase;

struct InterpolationResult {
  double weight_prev;
  double weight_after;
};

class Interpolator {
 public:
  enum FunctionType {
    none,
    linear,
    linearTime,
    exponentialSimple,
    exponentialSimpleTime,
    exponentialKernel
  };
  struct Config : public config_utilities::Config<Config> {
    int verbosity = 4;
    std::string func_name = "none";
    std::vector<double> function_config;
    Config() { setConfigName("interpolator"); }
    // static Config fromXmlRpcValue(XmlRpc::XmlRpcValue xml_value);
   protected:
    void setupParamsAndPrinting() override;
    void checkParams() const override;
  };
  static FunctionType FunctionTypeFromString(const std::string& s);
  static std::string FunctionTypeToString(const FunctionType ft);
  static std::unique_ptr<InterpolatorFunctionBase>
  FunctionTypeToInterpolatorFunction(
      const FunctionType ft, const std::vector<double>& function_config);
  explicit Interpolator(const Config& config, const ros::NodeHandle& nh);
  virtual ~Interpolator() = default;
  InterpolationResult interpolate(const V6_t& point_log_pose,
                                  const V6_t& start_log_pose,
                                  const V6_t& end_log_pose,
                                  const double point_time,
                                  const double start_time,
                                  const double end_time) const;

 private:
  Config config_;
  std::unique_ptr<InterpolatorFunctionBase> interpolator_;
  ros::NodeHandle nh_;
};

class InterpolatorFunctionBase {
 public:
  InterpolatorFunctionBase() {}
  explicit InterpolatorFunctionBase(const std::vector<double>& config_values) {}
  virtual ~InterpolatorFunctionBase() = default;
  virtual InterpolationResult interpolateSingle(
      const V6_t& point_log_pose, const V6_t& mid_log_pose,
      const double point_time, const double mid_time) const = 0;
  virtual InterpolationResult interpolateDouble(
      const V6_t& point_log_pose, const V6_t& start_log_pose,
      const V6_t& end_log_pose, const double point_time,
      const double start_time, const double end_time) const = 0;
};

class InterpolatorFunctionNone : public InterpolatorFunctionBase {
 public:
  explicit InterpolatorFunctionNone(const std::vector<double>& config_values) {
    double mean = config_values[0];
    double std = config_values[1];
  }
  InterpolationResult interpolateSingle(const V6_t& point_log_pose,
                                        const V6_t& mid_log_pose,
                                        const double point_time,
                                        const double mid_time) const {
    return {0.0, 0.0};
  }
  InterpolationResult interpolateDouble(const V6_t& point_log_pose,
                                        const V6_t& start_log_pose,
                                        const V6_t& end_log_pose,
                                        const double point_time,
                                        const double start_time,
                                        const double end_time) const {
    return {0.0, 0.0};
  }
};

class InterpolatorFunctionLinear : public InterpolatorFunctionBase {
 public:
  InterpolatorFunctionLinear() {}
  InterpolationResult interpolateSingle(const V6_t& point_log_pose,
                                        const V6_t& mid_log_pose,
                                        const double point_time,
                                        const double mid_time) const;
  InterpolationResult interpolateDouble(const V6_t& point_log_pose,
                                        const V6_t& start_log_pose,
                                        const V6_t& end_log_pose,
                                        const double point_time,
                                        const double start_time,
                                        const double end_time) const;

 private:
  double trivial_dist = 0.005;  // 5 cm
  double max_dist = 30;         // 30 m
};

class InterpolatorFunctionLinearTime : public InterpolatorFunctionBase {
 public:
  InterpolatorFunctionLinearTime() {}
  InterpolationResult interpolateSingle(const V6_t& point_log_pose,
                                        const V6_t& mid_log_pose,
                                        const double point_time,
                                        const double mid_time) const;
  InterpolationResult interpolateDouble(const V6_t& point_log_pose,
                                        const V6_t& start_log_pose,
                                        const V6_t& end_log_pose,
                                        const double point_time,
                                        const double start_time,
                                        const double end_time) const;

 private:
  double trivial_dtime = 0.01;  // 10 ms - or the system's latency
  double max_dtime = 20;        // 20 s for 20 fps is 400 frames
};

class InterpolatorFunctionExponentialSimple : public InterpolatorFunctionBase {
 public:
  explicit InterpolatorFunctionExponentialSimple(const double mean = 0.0,
                                                 const double sigma = 1.0)
      : mean_(mean), sigma_(sigma) {}
  InterpolationResult interpolateSingle(const V6_t& point_log_pose,
                                        const V6_t& mid_log_pose,
                                        const double point_time,
                                        const double mid_time) const;
  InterpolationResult interpolateDouble(const V6_t& point_log_pose,
                                        const V6_t& start_log_pose,
                                        const V6_t& end_log_pose,
                                        const double point_time,
                                        const double start_time,
                                        const double end_time) const;

 private:
  double mean_;
  double sigma_;
};

class InterpolatorFunctionExponentialSimpleTime
    : public InterpolatorFunctionBase {
 public:
  explicit InterpolatorFunctionExponentialSimpleTime(const double mean = 0.0,
                                                     const double sigma = 1.0)
      : mean_(mean), sigma_(sigma) {}
  InterpolationResult interpolateSingle(const V6_t& point_log_pose,
                                        const V6_t& mid_log_pose,
                                        const double point_time,
                                        const double mid_time) const;
  InterpolationResult interpolateDouble(const V6_t& point_log_pose,
                                        const V6_t& start_log_pose,
                                        const V6_t& end_log_pose,
                                        const double point_time,
                                        const double start_time,
                                        const double end_time) const;

 private:
  double mean_;
  double sigma_;
};

// class InterpolatorFunctionExponentialKernel : public InterpolatorFunctionBase
// {
//  public:
//   InterpolatorFunctionExponentialKernel(
//       const Transformation::Vector6 mean = Transformation::Vector6::Zero(),
//       const Eigen::MatrixXd sigma = Eigen::MatrixXd::Identity(6, 6))
//       : mean_(mean), sigma_(sigma) {}
//   Transformation::Vector6 interpolateSingle(
//       const Transformation::Vector6& point_log_pose,
//       const Transformation::Vector6& mid_log_pose, const double point_time,
//       const double mid_time) const;
//   Transformation::Vector6 interpolateDouble(
//       const Transformation::Vector6& point_log_pose,
//       const Transformation::Vector6& start_log_pose,
//       const Transformation::Vector6& end_log_pose, const double point_time,
//       const double start_time, const double end_time) const;

//  private:
//   Transformation::Vector6 mean_;
//   Eigen::MatrixXd sigma_;
// };

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_MANAGEMENT_INTERPOLATION_FUNCTION_H_
