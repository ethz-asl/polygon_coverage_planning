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
