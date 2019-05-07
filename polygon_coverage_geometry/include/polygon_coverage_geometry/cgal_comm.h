#include "polygon_coverage_geometry/cgal_definitions.h"

namespace polygon_coverage_planning {

// Helper to check whether a point is inside or on the boundary of the
// polygon.
bool pointInPolygon(const PolygonWithHoles& pwh, const Point_2& p);
bool pointsInPolygon(const PolygonWithHoles& pwh,
                     const std::vector<Point_2>::iterator& begin,
                     const std::vector<Point_2>::iterator& end);

}  // namespace polygon_coverage_planning
