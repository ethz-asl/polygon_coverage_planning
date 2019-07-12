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
#include "polygon_coverage_geometry/test_comm.h"

using namespace polygon_coverage_planning;

TEST(CgalCommTest, pointInPolygon) {
  PolygonWithHoles rect_in_rect(
      createRectangleInRectangle<Polygon_2, PolygonWithHoles>());

  // Point on vertex.
  Point_2 p(0.0, 0.0);
  EXPECT_TRUE(pointInPolygon(rect_in_rect, p));

  // Point on edge.
  Point_2 q(1.0, 0.0);
  EXPECT_TRUE(pointInPolygon(rect_in_rect, q));

  // Point in interior.
  Point_2 r(1.0, 1.0);
  EXPECT_TRUE(pointInPolygon(rect_in_rect, r));

  // Point on hole vertex.
  Point_2 s(1.0, 1.25);
  EXPECT_TRUE(pointInPolygon(rect_in_rect, s));

  // Point on hole edge.
  Point_2 t(0.75, 1.75);
  EXPECT_TRUE(pointInPolygon(rect_in_rect, t));

  // Point in hole.
  Point_2 u(0.75, 1.5);
  EXPECT_FALSE(pointInPolygon(rect_in_rect, u));

  // Point outside polygon.
  Point_2 v(100.0, 100.0);
  EXPECT_FALSE(pointInPolygon(rect_in_rect, v));
}

TEST(CgalCommTest, projectPointOnHull) {
  PolygonWithHoles poly(
      createRectangleInRectangle<Polygon_2, PolygonWithHoles>());

  // Point outside closest to vertex.
  Point_2 p(-1.0, -1.0);
  EXPECT_EQ(Point_2(0.0, 0.0), projectPointOnHull(poly, p));

  // Point inside closest to bottom edge.
  p = Point_2(0.1, 0.05);
  EXPECT_EQ(Point_2(0.1, 0.0), projectPointOnHull(poly, p));

  // Point inside hole closest to bottom edge.
  p = Point_2(0.75, 1.3);
  EXPECT_EQ(Point_2(0.75, 1.25), projectPointOnHull(poly, p));

  // Point inside polygon closest to hole bottom edge.
  p = Point_2(0.75, 1.2);
  EXPECT_EQ(Point_2(0.75, 1.25), projectPointOnHull(poly, p));

  // Point on edge.
  p = Point_2(0.5, 0);
  EXPECT_EQ(p, projectPointOnHull(poly, p));

  // Point on vertex.
  p = Point_2(0, 0);
  EXPECT_EQ(p, projectPointOnHull(poly, p));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
