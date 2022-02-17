#ifndef PANOPTIC_MAPPING_COMMON_BOUNDING_BOX_EXTENDED_H_
#define PANOPTIC_MAPPING_COMMON_BOUNDING_BOX_EXTENDED_H_

#include <algorithm>
#include <utility>

#include <eigen_conversions/eigen_msg.h>
#include <panoptic_mapping_msgs/BoundingBox.h>
#include <voxgraph/frontend/submap_collection/bounding_box.h>

#include "panoptic_mapping/common/common.h"

namespace panoptic_mapping {

typedef panoptic_mapping_msgs::BoundingBox BoundingBoxMsg;
class BoundingBoxExtended : public voxgraph::BoundingBox {
 public:
  BoundingBoxExtended() = default;
  ~BoundingBoxExtended() = default;
  void updateBoundingBoxLimits(const Point& p) {
    min.x() = std::min(p.x(), min.x());
    max.x() = std::max(p.x(), max.x());
    min.y() = std::min(p.y(), min.y());
    max.y() = std::max(p.y(), max.y());
    min.z() = std::min(p.z(), min.z());
    max.z() = std::max(p.z(), max.z());
  }
  /**
   * @brief Computes the intersection over Union between this box
   * and another one
   * @param other_bounding_box the other bounding box
   * @return float IoU metric
   */
  float IoU(voxgraph::BoundingBox* other_bounding_box) const {
    float union1 = computeVolume(max, min);
    float union2 =
        computeVolume(other_bounding_box->max, other_bounding_box->min);
    float xA = std::max(min.x(), other_bounding_box->min.x());
    float yA = std::max(min.y(), other_bounding_box->max.y());
    float zA = std::max(min.z(), other_bounding_box->max.z());
    float xB = std::min(max.x(), other_bounding_box->max.x());
    float yB = std::min(max.y(), other_bounding_box->max.y());
    float zB = std::min(max.z(), other_bounding_box->max.z());
    float interVolume =
        max(0, xB - xA + 1) * max(0, yB - yA + 1) * max(0, zB - zA + 1);
    float total_union = union1 + union2;
    assert(interVolume >= 0.0);
    assert(total_union >= 0.0);
    if (total_union == 0) {
      return 0.0;
    }
    return interVolume / total_union;
  }

  BoundingBoxMsg toBoundingBoxMsg() const {
    BoundingBoxMsg ret;
    tf::pointEigenToMsg(max.cast<double>(), ret.max);
    tf::pointEigenToMsg(min.cast<double>(), ret.min);
    return std::move(ret);
  }

  static BoundingBoxExtended fromMsg(const BoundingBoxMsg& msg) {
    BoundingBoxExtended be;
    Eigen::Vector3d max_d, min_d;
    tf::pointMsgToEigen(msg.max, max_d);
    tf::pointMsgToEigen(msg.min, min_d);
    be.max = max_d.cast<float>();
    be.min = min_d.cast<float>();
    return be;
  }

 protected:
  float computeVolume(const Point& max, const Point& min) const {
    return (max - min).prod();
  }
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_COMMON_BOUNDING_BOX_EXTENDED_H_
