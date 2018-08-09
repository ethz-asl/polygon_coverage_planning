#include <cstdlib>

#include <gtest/gtest.h>
#include <boost/make_shared.hpp>

#include "mav_2d_coverage_planning/polygon.h"

using namespace mav_coverage_planner;

double createRandomDouble(double min, double max) {
  return (max - min) * (static_cast<double>(std::rand()) /
                        static_cast<double>(RAND_MAX)) +
         min;
}

Polygon createRectangleInRectangle() {
  Polygon_2 outer;
  outer.push_back(Point_2(0.0, 0.0));
  outer.push_back(Point_2(2.0, 0.0));
  outer.push_back(Point_2(2.0, 2.0));
  outer.push_back(Point_2(0.0, 2.0));

  Polygon_2 hole;
  hole.push_back(Point_2(1.0, 1.25));
  hole.push_back(Point_2(1.0, 1.75));
  hole.push_back(Point_2(0.5, 1.75));
  hole.push_back(Point_2(0.5, 1.25));

  PolygonWithHoles poly_with_holes(outer);
  poly_with_holes.add_hole(hole);
  return Polygon(poly_with_holes);
}

Polygon createDiamond() {
  Polygon_2 outer;
  outer.push_back(Point_2(1.0, 0.0));
  outer.push_back(Point_2(2.0, 1.0));
  outer.push_back(Point_2(1.0, 2.0));
  outer.push_back(Point_2(0.0, 1.0));

  PolygonWithHoles poly(outer);
  return poly;
}

TEST(PolygonTest, Offset) {
  Polygon_2 outer;

  outer.push_back(Point_2(0.0, 0.0));
  outer.push_back(Point_2(10.0, 0.0));
  outer.push_back(Point_2(10.0, 4.5));
  outer.push_back(Point_2(12.0, 4.5));
  outer.push_back(Point_2(12.0, 2.0));
  outer.push_back(Point_2(16.0, 2.0));
  outer.push_back(Point_2(16.0, 8.0));
  outer.push_back(Point_2(12.0, 8.0));
  outer.push_back(Point_2(12.0, 5.5));
  outer.push_back(Point_2(10.0, 5.5));
  outer.push_back(Point_2(10.0, 10.0));
  outer.push_back(Point_2(0.0, 10.0));

  Polygon_2 hole;

  hole.push_back(Point_2(3.0, 3.0));
  hole.push_back(Point_2(3.0, 7.0));
  hole.push_back(Point_2(7.0, 7.0));
  hole.push_back(Point_2(7.0, 3.0));

  PolygonWithHoles poly_with_holes(outer);

  poly_with_holes.add_hole(hole);

  Polygon poly(poly_with_holes);

  for (size_t i = 0; i < 100; ++i) {
    FT max_offset = createRandomDouble(0.0, 10.0);
    Polygon offset_polygon;
    EXPECT_TRUE(poly.computeOffsetPolygon(max_offset, &offset_polygon));
    EXPECT_LE(offset_polygon.computeArea(), poly.computeArea());
  }
}

TEST(PolygonTest, ConvertPolygonWithHolesToPolygonWithoutHoles) {
  Polygon rectangle_in_rectangle = createRectangleInRectangle();
  Polygon poly_without_holes;
  EXPECT_TRUE(
      rectangle_in_rectangle.convertPolygonWithHolesToPolygonWithoutHoles(
          &poly_without_holes));
  EXPECT_EQ(0, poly_without_holes.getPolygon().number_of_holes());
  EXPECT_EQ(10, poly_without_holes.getPolygon().outer_boundary().size());
}

TEST(PolygonTest, ConvexDecomposition) {
  Polygon rectangle_in_rectangle = createRectangleInRectangle();
  std::vector<Polygon> convex_polygons;
  EXPECT_TRUE(
      rectangle_in_rectangle.computeConvexDecompositionFromPolygonWithHoles(
          &convex_polygons));
  EXPECT_EQ(4, convex_polygons.size());
}

TEST(PolygonTest, pointInPolygon) {
  Polygon rect_in_rect = createRectangleInRectangle();

  // Point on vertex.
  Point_2 p(0.0, 0.0);
  EXPECT_TRUE(rect_in_rect.pointInPolygon(p));

  // Point on edge.
  Point_2 q(1.0, 0.0);
  EXPECT_TRUE(rect_in_rect.pointInPolygon(q));

  // Point in interior.
  Point_2 r(1.0, 1.0);
  EXPECT_TRUE(rect_in_rect.pointInPolygon(r));

  // Point on hole vertex.
  Point_2 s(1.0, 1.25);
  EXPECT_TRUE(rect_in_rect.pointInPolygon(s));

  // Point on hole edge.
  Point_2 t(0.75, 1.75);
  EXPECT_TRUE(rect_in_rect.pointInPolygon(t));

  // Point in hole.
  Point_2 u(0.75, 1.5);
  EXPECT_FALSE(rect_in_rect.pointInPolygon(u));

  // Point outside polygon.
  Point_2 v(100.0, 100.0);
  EXPECT_FALSE(rect_in_rect.pointInPolygon(v));
}

TEST(PolygonTest, computeLineSweepPlan) {
  const double kMaxSweepDistance = 0.1;
  const double kStartVertexIdx = 0;
  const bool kCounterClockwise = true;

  Polygon diamond = createDiamond();
  std::vector<Point_2> waypoints;
  EXPECT_TRUE(diamond.computeLineSweepPlan(kMaxSweepDistance, kStartVertexIdx,
                                           kCounterClockwise, &waypoints));
  EXPECT_GE(waypoints.size(), 4);
  for (const Point_2& p : waypoints)
    EXPECT_TRUE(diamond.pointInPolygon(p));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
