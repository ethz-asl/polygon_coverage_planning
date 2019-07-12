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

#ifndef POLYGON_COVERAGE_MSGS_CONVERSION_H_
#define POLYGON_COVERAGE_MSGS_CONVERSION_H_

#include <geometry_msgs/Polygon.h>
#include <polygon_coverage_geometry/cgal_definitions.h>
#include <polygon_coverage_msgs/PolygonWithHoles.h>

namespace polygon_coverage_planning {

void convertPolygonWithHolesToMsg(const PolygonWithHoles& pwh,
                                  const double altitude,
                                  polygon_coverage_msgs::PolygonWithHoles* msg);

void convertPolygonToMsg(const Polygon_2& p, const double altitude,
                         geometry_msgs::Polygon* msg);

}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_MSGS_CONVERSION_H_
