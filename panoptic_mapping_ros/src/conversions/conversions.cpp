#include "panoptic_mapping_ros/conversions/conversions.h"

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
  return BoundingBoxExtended::fromMsg(msg);
}

PlaneType planeTypeFromMsg(const panoptic_mapping_msgs::PlaneType& msg) {
  return PlaneType::fromMsg(msg);
}

}  // namespace panoptic_mapping
