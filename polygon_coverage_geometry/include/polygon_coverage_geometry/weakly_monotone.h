#ifndef POLYGON_COVERAGE_GEOMETRY_WEAKLY_MONOTONE_H_
#define POLYGON_COVERAGE_GEOMETRY_WEAKLY_MONOTONE_H_

#include "polygon_coverage_geometry/cgal_definitions.h"

namespace polygon_coverage_planning {

// Check whether polygon 'in' is weakly monotone perpendicular to 'x_axis'.
bool isWeaklyMonotone(const Polygon_2& in, const Line_2& x_axis);
// For all edges check whether polygon 'in' is weakly monotone perpendicular to
// that edge.
std::vector<Direction_2> getAllSweepableEdgeDirections(const Polygon_2& in);

VertexConstCirculator findSouth(const Polygon_2& in, const Line_2& x_axis);
VertexConstCirculator findNorth(const Polygon_2& in, const Line_2& x_axis);

}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_GEOMETRY_WEAKLY_MONOTONE_H_
