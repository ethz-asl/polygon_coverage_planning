#include <cstdlib>

#include <gtest/gtest.h>
#include <boost/make_shared.hpp>

#include "mav_2d_coverage_planning/geometry/polygon.h"
#include "mav_2d_coverage_planning/tests/test_helpers.h"

using namespace mav_coverage_planning;

TEST(PolygonTest, Offset) {
  Polygon poly(createSophisticatedPolygon<Polygon_2, PolygonWithHoles>());

  for (size_t i = 0; i < 100; ++i) {
    FT max_offset = createRandomDouble(0.0, 10.0);
    Polygon offset_polygon;
    EXPECT_TRUE(poly.computeOffsetPolygon(max_offset, &offset_polygon));
    EXPECT_LE(offset_polygon.computeArea(), poly.computeArea());
  }
}

TEST(PolygonTest, ConvertPolygonWithHolesToPolygonWithoutHoles) {
  Polygon rectangle_in_rectangle(
      createRectangleInRectangle<Polygon_2, PolygonWithHoles>());
  Polygon poly_without_holes;
  EXPECT_TRUE(
      rectangle_in_rectangle.convertPolygonWithHolesToPolygonWithoutHoles(
          &poly_without_holes));
  EXPECT_EQ(0, poly_without_holes.getPolygon().number_of_holes());
  EXPECT_EQ(10, poly_without_holes.getPolygon().outer_boundary().size());
}

TEST(PolygonTest, ConvexDecomposition) {
  Polygon rectangle_in_rectangle(
      createRectangleInRectangle<Polygon_2, PolygonWithHoles>());
  std::vector<Polygon> convex_polygons;
  EXPECT_TRUE(
      rectangle_in_rectangle.computeConvexDecompositionFromPolygonWithHoles(
          &convex_polygons));
  EXPECT_EQ(4, convex_polygons.size());
}

TEST(PolygonTest, pointInOnPolygon) {
  Polygon rect_in_rect(
      createRectangleInRectangle<Polygon_2, PolygonWithHoles>());

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

  Polygon diamond(createDiamond<Polygon_2>());
  std::vector<Point_2> waypoints;
  EXPECT_TRUE(diamond.computeLineSweepPlan(kMaxSweepDistance, kStartVertexIdx,
                                           kCounterClockwise, &waypoints));
  EXPECT_GE(waypoints.size(), 4);
  for (const Point_2& p : waypoints) EXPECT_TRUE(diamond.pointInPolygon(p));
}

TEST(PolygonTest, computeVisibilityPolygon) {
  Polygon rectangle_in_rectangle(
      createRectangleInRectangle<Polygon_2, PolygonWithHoles>());
  Polygon visibility_polygon;

  // Query on polygon vertex.
  Point_2 query(0.0, 0.0);
  EXPECT_TRUE(rectangle_in_rectangle.computeVisibilityPolygon(
      query, &visibility_polygon));
  // Result manually checked.
  VertexConstIterator vit =
      visibility_polygon.getPolygon().outer_boundary().vertices_begin();
  EXPECT_EQ(9, visibility_polygon.getPolygon().outer_boundary().size());
  EXPECT_EQ(Point_2(0.0, 2.0), *vit++);
  EXPECT_EQ(Point_2(0.0, 0.0), *vit++);
  EXPECT_EQ(Point_2(2.0, 0.0), *vit++);
  EXPECT_EQ(Point_2(2.0, 2.0), *vit++);
  vit++;  // Skip inexact point. (1.6,2)
  EXPECT_EQ(Point_2(1.0, 1.25), *vit++);
  EXPECT_EQ(Point_2(0.5, 1.25), *vit++);
  EXPECT_EQ(Point_2(0.5, 1.75), *vit++);
  vit++;  // Skip inexact point. (0.571429,2)

  // Query on hole vertex.
  query = Point_2(1.0, 1.25);
  EXPECT_TRUE(rectangle_in_rectangle.computeVisibilityPolygon(
      query, &visibility_polygon));
  vit = visibility_polygon.getPolygon().outer_boundary().vertices_begin();
  EXPECT_EQ(6, visibility_polygon.getPolygon().outer_boundary().size());
  EXPECT_EQ(Point_2(1, 1.25), *vit++);
  EXPECT_EQ(Point_2(0, 1.25), *vit++);
  EXPECT_EQ(Point_2(0, 0), *vit++);
  EXPECT_EQ(Point_2(2, 0), *vit++);
  EXPECT_EQ(Point_2(2, 2), *vit++);
  EXPECT_EQ(Point_2(1, 2), *vit++);

  // Query in face.
  query = Point_2(1.0, 0.5);
  EXPECT_TRUE(rectangle_in_rectangle.computeVisibilityPolygon(
      query, &visibility_polygon));
  // Result manually checked.
  vit = visibility_polygon.getPolygon().outer_boundary().vertices_begin();
  EXPECT_EQ(7, visibility_polygon.getPolygon().outer_boundary().size());
  EXPECT_EQ(Point_2(1, 1.25), *vit++);
  EXPECT_EQ(Point_2(0.5, 1.25), *vit++);
  EXPECT_EQ(Point_2(0, 2), *vit++);
  EXPECT_EQ(Point_2(0, 0), *vit++);
  EXPECT_EQ(Point_2(2, 0), *vit++);
  EXPECT_EQ(Point_2(2, 2), *vit++);
  EXPECT_EQ(Point_2(1, 2), *vit++);

  // Query on polygon halfedge.
  query = Point_2(1.0, 0.0);
  EXPECT_TRUE(rectangle_in_rectangle.computeVisibilityPolygon(
      query, &visibility_polygon));
  // Result manually checked.
  vit = visibility_polygon.getPolygon().outer_boundary().vertices_begin();
  EXPECT_EQ(8, visibility_polygon.getPolygon().outer_boundary().size());
  EXPECT_EQ(Point_2(0, 2), *vit++);
  EXPECT_EQ(Point_2(0, 0), *vit++);
  EXPECT_EQ(Point_2(2, 0), *vit++);
  EXPECT_EQ(Point_2(2, 2), *vit++);
  EXPECT_EQ(Point_2(1, 2), *vit++);
  EXPECT_EQ(Point_2(1, 1.25), *vit++);
  EXPECT_EQ(Point_2(0.5, 1.25), *vit++);
  // EXPECT_EQ(Point_2(0.2, 2), *vit++); Skip inexact.

  // Query on hole halfedge.
  query = Point_2(0.75, 1.25);
  EXPECT_TRUE(rectangle_in_rectangle.computeVisibilityPolygon(
      query, &visibility_polygon));
  // Result manually checked.
  vit = visibility_polygon.getPolygon().outer_boundary().vertices_begin();
  EXPECT_EQ(4, visibility_polygon.getPolygon().outer_boundary().size());
  EXPECT_EQ(Point_2(2, 1.25), *vit++);
  EXPECT_EQ(Point_2(0, 1.25), *vit++);
  EXPECT_EQ(Point_2(0, 0), *vit++);
  EXPECT_EQ(Point_2(2, 0), *vit++);

  // Query outside.
  query = Point_2(100.0, 100.0);
  EXPECT_FALSE(rectangle_in_rectangle.computeVisibilityPolygon(
      query, &visibility_polygon));
}

TEST(PolygonTest, projectPointOnHull) {
  Polygon poly(createRectangleInRectangle<Polygon_2, PolygonWithHoles>());

  // Point outside closest to vertex.
  Point_2 p(-1.0, -1.0);
  EXPECT_EQ(Point_2(0.0, 0.0), poly.projectPointOnHull(p));

  // Point inside closest to bottom edge.
  p = Point_2(0.1, 0.05);
  EXPECT_EQ(Point_2(0.1, 0.0), poly.projectPointOnHull(p));

  // Point inside hole closest to bottom edge.
  p = Point_2(0.75, 1.3);
  EXPECT_EQ(Point_2(0.75, 1.25), poly.projectPointOnHull(p));

  // Point inside polygon closest to hole bottom edge.
  p = Point_2(0.75, 1.2);
  EXPECT_EQ(Point_2(0.75, 1.25), poly.projectPointOnHull(p));

  // Point on edge.
  p = Point_2(0.5, 0);
  EXPECT_EQ(p, poly.projectPointOnHull(p));

  // Point on vertex.
  p = Point_2(0, 0);
  EXPECT_EQ(p, poly.projectPointOnHull(p));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  return RUN_ALL_TESTS();
}
