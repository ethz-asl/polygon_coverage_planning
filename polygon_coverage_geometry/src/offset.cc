#include "polygon_coverage_geometry/offset.h"

#include <CGAL/create_offset_polygons_from_polygon_with_holes_2.h>
#include <ros/assert.h>
#include <boost/make_shared.hpp>

namespace polygon_coverage_planning {

void computeOffsetPolygon(const PolygonWithHoles& pwh, FT max_offset,
                          PolygonWithHoles* offset_polygon) {
  ROS_ASSERT(offset_polygon);

  // TODO(rikba): Check weak simplicity.

  // Try maximum offsetting.
  std::vector<boost::shared_ptr<PolygonWithHoles>> result =
      CGAL::create_interior_skeleton_and_offset_polygons_with_holes_2(
          max_offset, pwh);
  if (checkValidOffset(pwh, result)) {
    *offset_polygon = *result.front();
  } else {
    result = {boost::make_shared<PolygonWithHoles>(pwh)};
  }

  // Binary search for smaller valid offset.
  FT min = 0.0;
  FT max = max_offset;
  const FT kBinarySearchResolution = 0.1;
  while (max - min > kBinarySearchResolution) {
    const FT mid = (min + max) / 2.0;
    std::vector<boost::shared_ptr<PolygonWithHoles>> temp_result =
        CGAL::create_interior_skeleton_and_offset_polygons_with_holes_2(mid,
                                                                        pwh);
    if (checkValidOffset(pwh, temp_result)) {
      min = mid;
      result = temp_result;
    } else {
      max = mid;
    }
  }

  *offset_polygon = *result.front();
}

bool checkValidOffset(
    const PolygonWithHoles& original,
    const std::vector<boost::shared_ptr<PolygonWithHoles>>& offset) {
  // Valid if number of vertices remains constant.
  if (offset.size() != 1) return false;
  if (original.number_of_holes() != offset.front()->number_of_holes())
    return false;
  if (original.outer_boundary().size() !=
      offset.front()->outer_boundary().size())
    return false;

  PolygonWithHoles::Hole_const_iterator offset_hit =
      offset.front()->holes_begin();
  for (PolygonWithHoles::Hole_const_iterator
           original_hit = original.holes_begin();
       original_hit != original.holes_end(); ++original_hit, ++offset_hit) {
    if (original_hit->size() != offset_hit->size()) return false;
  }
  return true;
}

}  // namespace polygon_coverage_geometry
