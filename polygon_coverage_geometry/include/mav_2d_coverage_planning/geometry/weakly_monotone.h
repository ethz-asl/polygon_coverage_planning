#ifndef MAV_2D_COVERAGE_PLANNING_GEOMETRY_WEAKLY_MONOTONE_H_
#define MAV_2D_COVERAGE_PLANNING_GEOMETRY_WEAKLY_MONOTONE_H_

#include <mav_coverage_planning_comm/cgal_definitions.h>

namespace mav_coverage_planning {

bool isWeaklyMonotone(const Polygon_2& in, const Line_2& x_axis);
std::vector<Direction_2> getAllSweepableEdgeDirections(const Polygon_2& in);

VertexConstCirculator findSouth(const Polygon_2& in, const Line_2& x_axis);
VertexConstCirculator findNorth(const Polygon_2& in, const Line_2& x_axis);

}

#endif  // MAV_2D_COVERAGE_PLANNING_GEOMETRY_WEAKLY_MONOTONE_H_
