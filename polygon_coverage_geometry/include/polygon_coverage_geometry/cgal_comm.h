#include "polygon_coverage_geometry/cgal_definitions.h"

namespace polygon_coverage_planning {

// Helper to check whether a point is inside or on the boundary of the
// polygon.
bool pointInPolygon(const PolygonWithHoles& pwh, const Point_2& p);
inline bool pointInPolygon(const Polygon_2& poly, const Point_2& p) {
  return pointInPolygon(PolygonWithHoles(poly), p);
}
bool pointsInPolygon(const PolygonWithHoles& pwh,
                     const std::vector<Point_2>::iterator& begin,
                     const std::vector<Point_2>::iterator& end);

// Definition according to
// https://doc.cgal.org/latest/Straight_skeleton_2/index.html
bool isStrictlySimple(const PolygonWithHoles& pwh);

// Project a point on a polygon.
Point_2 projectOnPolygon2(const Polygon_2& poly, const Point_2& p,
                          FT* squared_distance);
// Project a point on the polygon boundary.
Point_2 projectPointOnHull(const PolygonWithHoles& pwh, const Point_2& p);

}  // namespace polygon_coverage_planning
