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

#ifndef POLYGON_COVERAGE_GEOMETRY_OFFSET_H_
#define POLYGON_COVERAGE_GEOMETRY_OFFSET_H_

#include "polygon_coverage_geometry/cgal_definitions.h"

namespace polygon_coverage_planning {

// Given a non-degenerate counter-clockwise weakly-simple polygon with
// holes, compute the maximum offset polygon such that no edge collapses.
// Aichholzer, Oswin, et al. "A novel type of skeleton for polygons." J. UCS
// The Journal of Universal Computer Science. Springer, Berlin, Heidelberg,
// 1996. 752-761.
void computeOffsetPolygon(const PolygonWithHoles& pwh, FT max_offset,
                          PolygonWithHoles* offset_polygon);

bool checkValidOffset(
    const PolygonWithHoles& original,
    const std::vector<boost::shared_ptr<PolygonWithHoles>>& offset);

// Offsets a specific polygon edge by cropping an along the edge infinitly
// long rectangular window with offset width.
bool offsetEdge(const Polygon_2& poly, const size_t& edge_id, double offset,
                Polygon_2* offset_polygon);

// Offset at most radial_offset from corner.
bool offsetEdgeWithRadialOffset(const Polygon_2& poly, const size_t& edge_id,
                                double radial_offset,
                                Polygon_2* offset_polygon);

}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_GEOMETRY_OFFSET_H_
