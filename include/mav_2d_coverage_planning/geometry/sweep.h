#ifndef MAV_2D_COVERAGE_PLANNING_GEOMETRY_SWEEP_H_
#define MAV_2D_COVERAGE_PLANNING_GEOMETRY_SWEEP_H_

#include <mav_coverage_planning_comm/cgal_definitions.h>

namespace mav_coverage_planning {
// Compute the sweep by moving from the bottom to the top of the polygon.
bool computeSweep(const Polygon_2& in, const FT offset, const Direction_2& dir,
                  bool counter_clockwise, std::vector<Point_2>* waypoints);

VertexConstIterator findApproxSouthEast(const Polygon_2& p);
VertexConstIterator findApproxSouthWest(const Polygon_2& p);
bool isApproxEqual(const FT a, const FT b);
std::vector<Point_2> findIntercections(const Polygon_2& p, const Line_2& l);

}  // namespace mav_coverage_planning

#endif  // MAV_2D_COVERAGE_PLANNING_GEOMETRY_SWEEP_H_
