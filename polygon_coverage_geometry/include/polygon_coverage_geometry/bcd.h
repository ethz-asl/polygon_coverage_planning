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

#ifndef POLYGON_COVERAGE_GEOMETRY_BCD_H_
#define POLYGON_COVERAGE_GEOMETRY_BCD_H_

#include "polygon_coverage_geometry/cgal_definitions.h"

// Choset, Howie. "Coverage of known spaces: The boustrophedon cellular
// decomposition." Autonomous Robots 9.3 (2000): 247-253.
// https://www.cs.cmu.edu/~motionplanning/lecture/Chap6-CellDecomp_howie.pdf
namespace polygon_coverage_planning {

std::vector<Polygon_2> computeBCD(const PolygonWithHoles& polygon_in,
                                  const Direction_2& dir);

}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_GEOMETRY_BCD_H_
