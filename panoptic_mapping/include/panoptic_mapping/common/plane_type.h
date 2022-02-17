#ifndef PANOPTIC_MAPPING_COMMON_PLANE_TYPE_H_
#define PANOPTIC_MAPPING_COMMON_PLANE_TYPE_H_

#include <vector>

#include <minkindr_conversions/kindr_msg.h>
#include <panoptic_mapping_msgs/PlaneType.h>

#include "panoptic_mapping/common/bounding_box_extended.h"
#include "panoptic_mapping/common/common.h"

namespace panoptic_mapping {

using BoundingBoxType = BoundingBoxExtended;

class PlaneType {
 public:
  typedef size_t PlaneID;
  explicit PlaneType(const Eigen::Vector3f& normal, const Point& point,
                     int class_id)
      : plane_(normal, normal.dot(point)),
        point_(point),
        class_id_(class_id),
        plane_id_(getNextPlaneID()) {
    assert(normal.squaredNorm() - 1 < 1.000001 &&
           1 - normal.squaredNorm() > 0.9999);
    buildPlaneOrientation();
    T_M_P_init_ = T_M_P_;
    num_points_ = 0;
  }
  explicit PlaneType(const Eigen::Hyperplane<float, 3>& plane, int class_id)
      : plane_(plane), class_id_(class_id), plane_id_(getNextPlaneID()) {
    const auto& n = plane.normal();
    point_ = plane.offset() * n;
    buildPlaneOrientation();
    T_M_P_init_ = T_M_P_;
    num_points_ = 0;
  }
  explicit PlaneType(const Eigen::Hyperplane<float, 3>& plane,
                     const Transformation& T_M_P, int class_id)
      : plane_(plane), class_id_(class_id), plane_id_(getNextPlaneID()) {
    const auto& n = plane.normal();
    point_ = plane.offset() * n;
    T_M_P_ = T_M_P;
    T_M_P_init_ = T_M_P;
    num_points_ = 0;
  }
  explicit PlaneType(const Eigen::Vector3f& normal, const Point& point,
                     const Transformation& T_M_P, int class_id)
      : PlaneType(Eigen::Hyperplane<float, 3>(normal, point), T_M_P, class_id) {
    assert(normal.squaredNorm() - 1 < 1.000001 &&
           1 - normal.squaredNorm() > 0.9999);
  }
  // getters
  Eigen::Vector3f getPlaneNormal() const {
    return plane_.normal().normalized();
  }
  Point getPointInit() const { return point_; }
  Transformation getPlaneTransformation() const { return T_M_P_; }
  PlaneID getPlaneID() const { return plane_id_; }
  const voxgraph::BoundingBox* getBoundingBox() const {
    return &planeSegmentAaBb_;
  }
  // transformation accessors
  /**
   * @brief Plane orientation is based on the world frame so a transformation
   * of the plane_ written in the mid pose's frame should respect that.
   *
   * @param Tnew_old
   */
  void transformPlane(const Transformation& Tnew_old,
                      const Transformation& T_M_R) {
    Transformation T_P_P;
    transformPlane(T_P_P);
  }
  void transformPlane(const Transformation& Tnew_old) {
    T_M_P_ = T_M_P_init_ * Tnew_old;
    plane_.transform(T_M_P_.getRotationMatrix(),
                     Eigen::TransformTraits::Isometry);
    plane_.offset() = plane_.normal().dot(T_M_P_.getPosition());
  }
  float distSquared(const PlaneType& other) const {
    return distFunc2(plane_.normal(), point_, other.plane_.normal(),
                     other.point_);
  }
  float dist(const PlaneType& other) const { return sqrt(distSquared(other)); }
  void buildPlaneOrientation() {
    Eigen::Matrix3f matRotation;
    const auto n = plane_.normal().normalized();
    const float& x = n.x();
    const float& y = n.y();
    const float& z = n.z();
    float sqrt_nx_ny = sqrt(x * x + y * y);
    matRotation << y / sqrt_nx_ny, -x / sqrt_nx_ny, 0.0, x * z / sqrt_nx_ny,
        y * z / sqrt_nx_ny, -sqrt_nx_ny, x, y, z;
    T_M_P_ = Transformation(point_, Transformation::Rotation(matRotation));
  }
  void createPlaneSegmentAaBb(const std::vector<Point>& points,
                              const double threshold_belongs) {
    num_points_ = 0;
    for (const Point& p : points) {
      if (plane_.absDistance(p) > threshold_belongs) {
        planeSegmentAaBb_.updateBoundingBoxLimits(p);
        ++num_points_;
      }
    }
  }
  void createPlaneSegmentAaBb(const std::vector<const Point*>& points,
                              const double threshold_belongs) {
    num_points_ = 0;
    for (const Point* p : points) {
      if (plane_.absDistance(*p) > threshold_belongs) {
        planeSegmentAaBb_.updateBoundingBoxLimits(*p);
        ++num_points_;
      }
    }
  }

  void setplaneSegmentAaBb(const BoundingBoxType& bbox) {
    planeSegmentAaBb_.min = bbox.min;
    planeSegmentAaBb_.max = bbox.max;
  }

  static float distFunc2(const Point& p1, const Point& n1, const Point& p2,
                         const Point& n2) {
    double d1 = (n2 - n1).squaredNorm();
    double d2 = (p2 - p1).dot(n2 + n1);
    return d1 + 0.5 * d2 * d2;
  }
  panoptic_mapping_msgs::PlaneType toPlaneTypeMsg() const {
    panoptic_mapping_msgs::PlaneType msg;
    msg.bbox = planeSegmentAaBb_.toBoundingBoxMsg();
    const Eigen::Vector3d point_d = point_.cast<double>();
    const Eigen::Vector3d normal_d = plane_.normal().cast<double>();
    msg.class_id = class_id_;
    tf::pointEigenToMsg(point_d, msg.point);
    tf::pointEigenToMsg(normal_d, msg.normal);
    tf::poseKindrToMsg(T_M_P_init_.cast<double>(), &msg.T_M_P);
    msg.num_points_ = num_points_;
    return msg;
  }
  static PlaneType fromMsg(const panoptic_mapping_msgs::PlaneType& msg) {
    Eigen::Vector3d n_d, p_d;
    tf::pointMsgToEigen(msg.normal, n_d);
    tf::pointMsgToEigen(msg.point, p_d);
    const Point n = n_d.cast<float>();
    const Point p = p_d.cast<float>();
    const int class_id = msg.class_id;
    const size_t num_points = msg.num_points_;
    kindr::minimal::QuatTransformationTemplate<double> tf_received;
    tf::poseMsgToKindr(msg.T_M_P, &tf_received);
    const Transformation T_M_P = tf_received.cast<float>();
    const PlaneType ret(n, p, T_M_P, class_id);
    return ret;
  }

 private:
  PlaneID plane_id_;
  static PlaneID getNextPlaneID() {
    static int next_plane_id = 0;
    return next_plane_id++;
  }
  int class_id_;
  Transformation T_M_P_;
  Transformation T_M_P_init_;
  Point point_;
  Eigen::Hyperplane<float, 3> plane_;
  size_t num_points_;
  BoundingBoxType planeSegmentAaBb_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_COMMON_PLANE_TYPE_H_
