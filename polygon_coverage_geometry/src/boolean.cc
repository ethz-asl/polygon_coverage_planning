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

#include "polygon_coverage_geometry/boolean.h"
#include "polygon_coverage_geometry/cgal_definitions.h"

#include <CGAL/General_polygon_set_2.h>
#include <CGAL/Gps_segment_traits_2.h>
#include <ros/assert.h>
#include <ros/console.h>

namespace polygon_coverage_planning {

std::list<PolygonWithHoles> computeDifference(
    const std::list<Polygon_2>::const_iterator& hull,
    const std::list<Polygon_2>::const_iterator& holes_begin,
    const std::list<Polygon_2>::const_iterator& holes_end) {
  return computeDifference(*hull, holes_begin, holes_end);
}

std::list<PolygonWithHoles> computeDifference(
    const Polygon_2& hull,
    const std::list<Polygon_2>::const_iterator& holes_begin,
    const std::list<Polygon_2>::const_iterator& holes_end) {
  typedef CGAL::Gps_segment_traits_2<K> Traits_2;
  typedef CGAL::General_polygon_set_2<Traits_2> Polygon_set_2;

  Polygon_set_2 gps(hull);
  for (auto h = holes_begin; h != holes_end; ++h) {
    gps.difference(*h);
  }

  std::list<PolygonWithHoles> res;
  gps.polygons_with_holes(std::back_inserter(res));
  return res;
}

}  // namespace polygon_coverage_planning
