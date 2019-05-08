#ifndef POLYGON_COVERAGE_GEOMETRY_VISIBILITY_POLYGON_H_
#define POLYGON_COVERAGE_GEOMETRY_VISIBILITY_POLYGON_H_

#include "polygon_coverage_geometry/cgal_definitions.h"

namespace polygon_coverage_planning {

// Compute the visibility polygon given a point inside a strictly simple
// polygon. Francisc Bungiu, Michael Hemmer, John Hershberger, Kan Huang, and
// Alexander Kröller. Efficient computation of visibility polygons. CoRR,
// abs/1403.3905, 2014.
bool computeVisibilityPolygon(const PolygonWithHoles& pwh,
                              const Point_2& query_point,
                              Polygon_2* visibility_polygon);

}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_GEOMETRY_VISIBILITY_POLYGON_H_
