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

#include <gtest/gtest.h>

#include "polygon_coverage_geometry/cgal_comm.h"
#include "polygon_coverage_geometry/offset.h"
#include "polygon_coverage_geometry/test_comm.h"

using namespace polygon_coverage_planning;

TEST(OffsetTest, OffsetPolygon) {
  PolygonWithHoles poly =
      createSophisticatedPolygon<Polygon_2, PolygonWithHoles>();

  for (size_t i = 0; i < 100; ++i) {
    FT max_offset = createRandomDouble(0.0, 10.0);
    PolygonWithHoles offset_polygon;
    computeOffsetPolygon(poly, max_offset, &offset_polygon);
    EXPECT_LE(computeArea(offset_polygon), computeArea(poly));
  }
}

TEST(OffsetTest, OffsetEdge) {
  PolygonWithHoles rectangle_in_rectangle(
      createRectangleInRectangle<Polygon_2, PolygonWithHoles>());
  Polygon_2 rectangle(rectangle_in_rectangle.outer_boundary());
  const double kOffset = 0.1;
  double area = CGAL::to_double(computeArea(rectangle));
  for (size_t i = 0; i < 4; ++i) {
    Polygon_2 offsetted_polygon;
    EXPECT_TRUE(offsetEdge(rectangle, i, kOffset, &offsetted_polygon));
    double area_offsetted = CGAL::to_double(computeArea(offsetted_polygon));
    double expected_difference =
        kOffset *
        std::sqrt(CGAL::to_double(rectangle.edge(i).squared_length()));
    const double kPrecision = 1.0e-3;
    EXPECT_LE(area - expected_difference - area_offsetted, kPrecision)
        << offsetted_polygon;
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
