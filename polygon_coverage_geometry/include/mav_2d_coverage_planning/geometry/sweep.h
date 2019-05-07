#ifndef MAV_2D_COVERAGE_PLANNING_GEOMETRY_SWEEP_H_
#define MAV_2D_COVERAGE_PLANNING_GEOMETRY_SWEEP_H_

#include <mav_coverage_planning_comm/cgal_definitions.h>
#include "mav_2d_coverage_planning/geometry/polygon.h"
#include "mav_2d_coverage_planning/graphs/visibility_graph.h"

namespace mav_coverage_planning {
// Compute the sweep by moving from the bottom to the top of the polygon.
bool computeSweep(const Polygon_2& in,
                  const visibility_graph::VisibilityGraph& visibility_graph,
                  const FT offset, const Direction_2& dir,
                  bool counter_clockwise, std::vector<Point_2>* waypoints);

// Compute sweeps in all sweepable directions, starting counter-clockwise,
// clockwise, and reverse.
bool computeAllSweeps(const Polygon& polygon, const double max_sweep_offset,
                      std::vector<std::vector<Point_2>>* cluster_sweeps);

// A segment is observable if all vertices between two sweeps are observable.
void checkObservability(
    const Segment_2& prev_sweep, const Segment_2& sweep,
    const std::vector<Point_2>& sorted_pts, const FT max_sq_distance,
    std::vector<Point_2>::const_iterator* lowest_unobservable_point);

// Find the intersections between a polygon and a line and sort them by the
// distance to the perpendicular direction of the line.
std::vector<Point_2> findIntersections(const Polygon_2& p, const Line_2& l);

// Same as findIntersections but only return first and last intersection.
bool findSweepSegment(const Polygon_2& p, const Line_2& l,
                      Segment_2* sweep_segment);

// Sort vertices of polygon based on signed distance to line l.
std::vector<Point_2> sortVerticesToLine(const Polygon_2& p, const Line_2& l);

// Connect to points in the polygon using the visibility graph.
bool calculateShortestPath(
    const visibility_graph::VisibilityGraph& visibility_graph,
    const Point_2& start, const Point_2& goal,
    std::vector<Point_2>* shortest_path);

}  // namespace mav_coverage_planning

#endif  // MAV_2D_COVERAGE_PLANNING_GEOMETRY_SWEEP_H_
