/*
 * polygon_coverage_planning implements algorithms for coverage planning in
 * general polygons with holes. Copyright (C) 2019, Rik Bähnemann, Autonomous
 * Systems Lab, ETH Zürich
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef POLYGON_COVERAGE_GEOMETRY_SWEEP_H_
#define POLYGON_COVERAGE_GEOMETRY_SWEEP_H_

#include "polygon_coverage_geometry/cgal_definitions.h"
#include "polygon_coverage_geometry/visibility_graph.h"

namespace polygon_coverage_planning {

// Compute the sweep by moving from the bottom to the top of the polygon.
bool computeSweep(const Polygon_2& in,
                  const visibility_graph::VisibilityGraph& visibility_graph,
                  const FT offset, const Direction_2& dir,
                  bool counter_clockwise, std::vector<Point_2>* waypoints);

// Compute sweeps in all sweepable directions, starting counter-clockwise,
// clockwise, and reverse.
bool computeAllSweeps(const Polygon_2& poly, const double max_sweep_offset,
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

}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_GEOMETRY_SWEEP_H_
