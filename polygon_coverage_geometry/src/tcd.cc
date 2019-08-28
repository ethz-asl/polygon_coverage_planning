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

#include "polygon_coverage_geometry/cgal_comm.h"
#include "polygon_coverage_geometry/cgal_definitions.h"
#include "polygon_coverage_geometry/tcd.h"

#include <CGAL/Polygon_vertical_decomposition_2.h>

namespace polygon_coverage_planning {

std::vector<Polygon_2> computeTCD(const PolygonWithHoles& polygon_in,
                                  const Direction_2& dir) {
  // Rotate polygon to have direction aligned with x-axis.
  // TODO(rikba): Make this independent of rotation.
  PolygonWithHoles rotated_polygon = rotatePolygon(polygon_in, dir);

  // TCD
  std::vector<Polygon_2> traps;
  CGAL::Polygon_vertical_decomposition_2<K> decom;
  decom(rotated_polygon, std::back_inserter(traps));

  // Rotate back all polygons.
  for (auto& p : traps) {
    CGAL::Aff_transformation_2<K> rotation(CGAL::ROTATION, dir, 1, 1e9);
    p = CGAL::transform(rotation, p);
  }

  return traps;
}

}  // namespace polygon_coverage_planning
