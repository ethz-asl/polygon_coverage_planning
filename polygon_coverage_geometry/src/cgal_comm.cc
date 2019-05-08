#include "polygon_coverage_geometry/cgal_comm.h"

namespace polygon_coverage_planning {

bool pointInPolygon(const PolygonWithHoles& pwh, const Point_2& p) {
  // Point inside outer boundary.
  CGAL::Bounded_side result =
      CGAL::bounded_side_2(pwh.outer_boundary().vertices_begin(),
                           pwh.outer_boundary().vertices_end(), p, K());
  if (result == CGAL::ON_UNBOUNDED_SIDE) return false;

  // Point outside hole.
  for (PolygonWithHoles::Hole_const_iterator hit = pwh.holes_begin();
       hit != pwh.holes_end(); ++hit) {
    result = CGAL::bounded_side_2(hit->vertices_begin(), hit->vertices_end(), p,
                                  K());
    if (result == CGAL::ON_BOUNDED_SIDE) return false;
  }

  return true;
}

bool pointsInPolygon(const PolygonWithHoles& pwh,
                     const std::vector<Point_2>::iterator& begin,
                     const std::vector<Point_2>::iterator& end) {
  for (std::vector<Point_2>::iterator it = begin; it != end; ++it) {
    if (!pointInPolygon(pwh, *it)) return false;
  }
  return true;
}

bool isStrictlySimple(const PolygonWithHoles& pwh) {
  for (PolygonWithHoles::Hole_const_iterator hi = pwh.holes_begin();
       hi != pwh.holes_end(); ++hi)
    if (!hi->is_simple()) return false;
  return pwh.outer_boundary().is_simple();
}

}  // namespace polygon_coverage_planning
