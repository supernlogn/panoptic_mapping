#ifndef PANOPTIC_MAPPING_COMMON_PLANE_TYPE_H_
#define PANOPTIC_MAPPING_COMMON_PLANE_TYPE_H_

#include <vector>

#include "panoptic_mapping/common/bounding_box_extended.h"
#include "panoptic_mapping/common/common.h"

namespace panoptic_mapping {

using BoundingBoxType = BoundingBoxExtended;

class PlaneType {
 public:
  typedef size_t PlaneID;
  explicit PlaneType(const Eigen::Vector3f& normal,
                     const Eigen::Vector3f& point, int class_id)
      : plane(normal, normal * point),
        point_(point),
        class_id_(class_id),
        plane_id_(getNextPlaneID()) {
    assert(normal.squaredNorm() - 1 < 1.000001 &&
           1 - normal.squaredNorm() > 0.9999);
    buildPlaneOrientation();
    T_M_P_init_ = T_M_P_;
    num_points_ = 0;
  }
  // getters
  Eigen::Vector3f getPlaneNormal() const { return plane.normal(); }
  Eigen::Vector3f getPointInit() const { return point_; }
  Transformation getPlaneTransformation() const { return T_M_P_; }
  PlaneID getPlaneID() const { return plane_id_; }
  const voxgraph::BoundingBox* getBoundingBox() const {
    return &planeSegmentAaBb_;
  }
  // transformation accessors
  /**
   * @brief Plane orientation is based on the world frame so a transformation
   * of the plane written in the mid pose's frame should respect that.
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
    plane.transform(T_M_P_.getRotationMatrix(),
                    Eigen::TransformTraits::Isometry);
    plane.offset() = plane.normal().dot(T_M_P_.getPosition());
  }
  double dist(const PlaneType& other) const {
    double d1 = (other.plane.normal() - plane.normal()).squaredNorm();
    double d2_ = (other.plane.offset() - plane.offset());
    double d2 = d2_ * d2_;
    return d1 + 0.5 * d2_;
  }
  void buildPlaneOrientation() {
    Eigen::Matrix3f matRotation;
    Eigen::Vector3f n = plane.normal();
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
      if (plane.absDistance(p) > threshold_belongs) {
        planeSegmentAaBb_.updateBoundingBoxLimits(p);
        ++num_points_
      }
    }
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
  Transformation::Vector3 point_;
  Eigen::Hyperplane<float, 3> plane;
  size_t num_points_;
  BoundingBoxType planeSegmentAaBb_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_COMMON_PLANE_TYPE_H_
