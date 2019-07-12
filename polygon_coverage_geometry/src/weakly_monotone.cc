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

#include "polygon_coverage_geometry/weakly_monotone.h"

namespace polygon_coverage_planning {

bool isWeaklyMonotone(const Polygon_2& in, const Line_2& x_axis) {
  // Find north and south.
  VertexConstCirculator north = findNorth(in, x_axis);
  VertexConstCirculator south = findSouth(in, x_axis);

  // Go from south to north vertex.
  VertexConstCirculator c = south;
  VertexConstCirculator c_prev = south;
  for (c++; c != north; c++) {
    if (CGAL::has_smaller_signed_distance_to_line(x_axis, *c, *c_prev))
      return false;
    c_prev = c;
  }

  // Go opposite direction.
  c = south;
  c_prev = south;
  for (c--; c != north; c--) {
    if (CGAL::has_smaller_signed_distance_to_line(x_axis, *c, *c_prev))
      return false;
    c_prev = c;
  }

  return true;
}

std::vector<Direction_2> getAllSweepableEdgeDirections(const Polygon_2& in) {
  // Get all directions.
  std::vector<Direction_2> dirs;
  for (EdgeConstIterator it = in.edges_begin(); it != in.edges_end(); ++it) {
    // Check if this edge direction is already in the set.
    std::vector<Direction_2>::iterator last =
        std::find_if(dirs.begin(), dirs.end(), [&it](const Direction_2& dir) {
          return CGAL::orientation(dir.vector(), it->to_vector()) ==
                 CGAL::COLLINEAR;
        });
    if (last != dirs.end()) continue;
    // Check if the polygon is monotone perpendicular to this edge direction.
    if (isWeaklyMonotone(in, it->supporting_line()))
      dirs.push_back(it->direction());
  }

  return dirs;
}

VertexConstCirculator findSouth(const Polygon_2& in, const Line_2& x_axis) {
  VertexConstCirculator vc = in.vertices_circulator();
  VertexConstCirculator v = vc;
  do {
    v = CGAL::has_smaller_signed_distance_to_line(x_axis, *vc, *v) ? vc : v;
  } while (++vc != in.vertices_circulator());
  return v;
}

VertexConstCirculator findNorth(const Polygon_2& in, const Line_2& x_axis) {
  return findSouth(in, x_axis.opposite());
}

}  // namespace polygon_coverage_planning
