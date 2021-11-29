#ifndef PANOPTIC_MAPPING_MAP_PSEUDO_SUBMAP_BOUNDING_VOLUME_H_
#define PANOPTIC_MAPPING_MAP_PSEUDO_SUBMAP_BOUNDING_VOLUME_H_

#include "panoptic_mapping/common/common.h"
#include "panoptic_mapping/map/submap_bounding_volume.h"

namespace panoptic_mapping {

class PseudoSubmap;

/**
 * This class interfaces conservative bounding volumes to hierarchically
 * prune submaps. Implemented using an inexact conservative sphere
 * approximation. The bounding volume is owned by the submap it references.
 */
class PseudoSubmapBoundingVolume {
 public:
  PseudoSubmapBoundingVolume():submap_(nullptr) {}
  explicit PseudoSubmapBoundingVolume(const PseudoSubmap& submap);
  explicit PseudoSubmapBoundingVolume(const PseudoSubmap& submap, 
                const SubmapBoundingVolume & submapBoundingVolume);
  ~PseudoSubmapBoundingVolume() = default;

  // copy ctor
  void clone(PseudoSubmapBoundingVolume & other) const;
  
  // update
  void update();

  // Interaction.
  bool contains_S(const Point& point_S) const;
  bool contains_M(const Point& point_M) const;
  bool intersects(const PseudoSubmapBoundingVolume& other) const;
  bool isInsidePlane_S(const Point& normal_S) const;
  bool isInsidePlane_M(const Point& normal_M) const;
  
  // Access.
  FloatingPoint getRadius() const { return radius_; }
  const Point& getCenter() const { return center_; }
  
 private:
  const PseudoSubmap* submap_;
  Point center_;  // This is in submap frame.
  FloatingPoint radius_;
  size_t num_previous_blocks_;
};

}  // namespace panoptic_mapping

#endif  // PANOPTIC_MAPPING_MAP_SUBMAP_BOUNDING_VOLUME_H_
