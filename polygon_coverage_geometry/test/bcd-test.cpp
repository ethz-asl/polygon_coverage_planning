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

#include <CGAL/is_y_monotone_2.h>
#include <gtest/gtest.h>

#include "polygon_coverage_geometry/bcd.h"
#include "polygon_coverage_geometry/cgal_comm.h"
#include "polygon_coverage_geometry/test_comm.h"

using namespace polygon_coverage_planning;

TEST(BctTest, computeBCD) {
  // Diamond.
  PolygonWithHoles diamond(createDiamond<Polygon_2>());
  FT expected_area = diamond.outer_boundary().area();
  std::vector<Polygon_2> bcd = computeBCD(diamond, Direction_2(1, 0));
  EXPECT_EQ(bcd.size(), static_cast<size_t>(1));
  EXPECT_EQ(bcd[0].size(), diamond.outer_boundary().size()) << bcd[0];
  FT area = 0.0;
  for (const Polygon_2& p : bcd) {
    EXPECT_TRUE(CGAL::is_y_monotone_2(p.vertices_begin(), p.vertices_end()));
    EXPECT_EQ(p.size(), static_cast<size_t>(4));
    area += p.area();
  }
  EXPECT_EQ(area, expected_area);

  // Rectangle in rectangle.
  PolygonWithHoles rectangle_in_rectangle(
      createRectangleInRectangle<Polygon_2, PolygonWithHoles>());
  expected_area = rectangle_in_rectangle.outer_boundary().area();
  for (PolygonWithHoles::Hole_const_iterator hit =
           rectangle_in_rectangle.holes_begin();
       hit != rectangle_in_rectangle.holes_end(); ++hit) {
    expected_area -= hit->area();
  }
  bcd = computeBCD(rectangle_in_rectangle, Direction_2(1, 0));
  EXPECT_EQ(bcd.size(), static_cast<size_t>(4));
  area = 0.0;
  for (const Polygon_2& p : bcd) {
    EXPECT_TRUE(CGAL::is_y_monotone_2(p.vertices_begin(), p.vertices_end()));
    EXPECT_EQ(p.size(), static_cast<size_t>(4));
    area += p.area();
  }
  EXPECT_EQ(area, expected_area);

  // Ultimate test.
  PolygonWithHoles pwh(createUltimateBCDTest<Polygon_2, PolygonWithHoles>());
  expected_area = pwh.outer_boundary().area();

  for (PolygonWithHoles::Hole_const_iterator hit = pwh.holes_begin();
       hit != pwh.holes_end(); ++hit) {
    expected_area += hit->area();
  }
  bcd = computeBCD(pwh, Direction_2(1, 0));
  EXPECT_EQ(bcd.size(), static_cast<size_t>(14));
  area = 0.0;
  for (const Polygon_2& p : bcd) {
    Direction_2 dir(0, 1);
    CGAL::Aff_transformation_2<K> rotation(CGAL::ROTATION, dir, 1, 1e9);
    Polygon_2 rot_p = CGAL::transform(rotation, p);
    EXPECT_TRUE(
        CGAL::is_y_monotone_2(rot_p.vertices_begin(), rot_p.vertices_end()));
    area += p.area();
  }
  EXPECT_EQ(area, expected_area);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
