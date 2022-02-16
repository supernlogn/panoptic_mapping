#include "panoptic_mapping_ros/conversions/conversions.h"

#include "eigen_conversions/eigen_msg.h"

namespace panoptic_mapping {

DetectronLabel detectronLabelFromMsg(
    const panoptic_mapping_msgs::DetectronLabel& msg) {
  DetectronLabel result;
  result.id = msg.id;
  result.instance_id = msg.instance_id;
  result.is_thing = msg.is_thing;
  result.category_id = msg.category_id;
  result.score = msg.score;
  return result;
}

DetectronLabels detectronLabelsFromMsg(
    const panoptic_mapping_msgs::DetectronLabels& msg) {
  DetectronLabels result;
  for (const panoptic_mapping_msgs::DetectronLabel& label : msg.labels) {
    result[label.id] = detectronLabelFromMsg(label);
  }
  return result;
}

BoundingBoxExtended boundingBoxExtendedFromMsg(
    const panoptic_mapping_msgs::BoundingBox& msg) {
  BoundingBoxExtended be;
  Eigen::Vector3d max_d, min_d;
  tf::pointMsgToEigen(msg.max, max_d);
  tf::pointMsgToEigen(msg.min, min_d);
  be.max = Eigen::Vector3f(max_d);
  be.min = Eigen::Vector3f(min_d);
  return be;
}

}  // namespace panoptic_mapping
