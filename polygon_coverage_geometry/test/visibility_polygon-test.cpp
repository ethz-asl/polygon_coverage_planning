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

#include "polygon_coverage_geometry/visibility_polygon.h"

#include <CGAL/intersections.h>
#include <CGAL/is_y_monotone_2.h>
#include <gtest/gtest.h>

#include "polygon_coverage_geometry/cgal_comm.h"
#include "polygon_coverage_geometry/test_comm.h"

using namespace polygon_coverage_planning;

TEST(VisibilityPolygonTest, computeVisibilityPolygon) {
  PolygonWithHoles rectangle_in_rectangle(
      createRectangleInRectangle<Polygon_2, PolygonWithHoles>());
  Polygon_2 visibility_polygon;

  // Query on polygon vertex.
  Point_2 query(0, 0);
  EXPECT_TRUE(computeVisibilityPolygon(rectangle_in_rectangle, query,
                                       &visibility_polygon));
  // Result manually checked.
  Line_2 line(Point_2(0, 2), Point_2(2, 2));
  Ray_2 ray_1(Point_2(0, 0), Point_2(0.5, 1.75));
  Ray_2 ray_2(Point_2(0, 0), Point_2(1.0, 1.25));
  auto inter_1 = CGAL::intersection(ray_1, line);
  auto inter_2 = CGAL::intersection(ray_2, line);

  Point_2 points_1[] = {{0, 2},
                        {0, 0},
                        {2, 0},
                        {2, 2},
                        *boost::get<Point_2>(&*inter_2),
                        {1, 1.25},
                        {0.5, 1.25},
                        {0.5, 1.75},
                        *boost::get<Point_2>(&*inter_1)};
  Polygon_2 visibility_polygon_expected_1(
      points_1, points_1 + sizeof(points_1) / sizeof(points_1[0]));
  EXPECT_EQ(visibility_polygon_expected_1, visibility_polygon);

  // Query on hole vertex.
  query = Point_2(1, 1.25);
  EXPECT_TRUE(computeVisibilityPolygon(rectangle_in_rectangle, query,
                                       &visibility_polygon));
  Point_2 points_2[] = {{0, 0}, {2, 0}, {2, 2}, {1, 2}, {1, 1.25}, {0, 1.25}};
  Polygon_2 visibility_polygon_expected_2(
      points_2, points_2 + sizeof(points_2) / sizeof(points_2[0]));
  EXPECT_EQ(visibility_polygon_expected_2, visibility_polygon);

  // Query in face.
  query = Point_2(1, 0.5);
  EXPECT_TRUE(computeVisibilityPolygon(rectangle_in_rectangle, query,
                                       &visibility_polygon));
  // Result manually checked.
  Point_2 points_3[] = {{0, 0},    {2, 0},      {2, 2}, {1, 2},
                        {1, 1.25}, {0.5, 1.25}, {0, 2}};
  Polygon_2 visibility_polygon_expected_3(
      points_3, points_3 + sizeof(points_3) / sizeof(points_3[0]));
  EXPECT_EQ(visibility_polygon_expected_3, visibility_polygon);

  // Query on polygon halfedge.
  query = Point_2(1, 0);
  EXPECT_TRUE(computeVisibilityPolygon(rectangle_in_rectangle, query,
                                       &visibility_polygon));
  // Result manually checked.
  Ray_2 ray_3(Point_2(1, 0), Point_2(0.5, 1.25));
  auto inter_3 = CGAL::intersection(ray_3, line);
  Point_2 points_4[] = {
      {0, 2}, {0, 0},    {2, 0},      {2, 2},
      {1, 2}, {1, 1.25}, {0.5, 1.25}, *boost::get<Point_2>(&*inter_3)};
  Polygon_2 visibility_polygon_expected_4(
      points_4, points_4 + sizeof(points_4) / sizeof(points_4[0]));
  EXPECT_EQ(visibility_polygon_expected_4, visibility_polygon);

  // Query on hole halfedge.
  query = Point_2(0.75, 1.25);
  EXPECT_TRUE(computeVisibilityPolygon(rectangle_in_rectangle, query,
                                       &visibility_polygon));
  // Result manually checked.
  Point_2 points_5[] = {{2, 1.25}, {0, 1.25}, {0, 0}, {2, 0}};
  Polygon_2 visibility_polygon_expected_5(
      points_5, points_5 + sizeof(points_5) / sizeof(points_5[0]));
  EXPECT_EQ(visibility_polygon_expected_5, visibility_polygon);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
