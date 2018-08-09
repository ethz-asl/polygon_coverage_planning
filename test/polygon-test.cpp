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

TEST(PolygonTest, pointInOnPolygon) {
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

TEST(PolygonTest, computeVisibilityPolygon) {
  Polygon rectangle_in_rectangle = createRectangleInRectangle();
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
  vit++; // Skip inexact point. (1.6,2)
  EXPECT_EQ(Point_2(1.0, 1.25), *vit++);
  EXPECT_EQ(Point_2(0.5, 1.25), *vit++);
  EXPECT_EQ(Point_2(0.5, 1.75), *vit++);
  vit++; // Skip inexact point. (0.571429,2)

  // Query on hole vertex.
  query = Point_2(1.0, 1.25);
  std::cout << "Query on hole vertex." << std::endl;
  rectangle_in_rectangle.print();
  EXPECT_TRUE(rectangle_in_rectangle.computeVisibilityPolygon(
      query, &visibility_polygon));
  visibility_polygon.print();

  // Query in face.
  query = Point_2(0.5, 0.5);
  std::cout << "Query in face." << std::endl;
  rectangle_in_rectangle.print();
  EXPECT_TRUE(rectangle_in_rectangle.computeVisibilityPolygon(
      query, &visibility_polygon));
  visibility_polygon.print();

  // Query on polygon halfedge.
  query = Point_2(1.0, 0.0);
  EXPECT_TRUE(rectangle_in_rectangle.computeVisibilityPolygon(
      query, &visibility_polygon));
  visibility_polygon.print();

  // Query on hole halfedge.
  query = Point_2(0.75, 1.25);
  EXPECT_TRUE(rectangle_in_rectangle.computeVisibilityPolygon(
      query, &visibility_polygon));
  visibility_polygon.print();

  // Query outside.
  query = Point_2(100.0, 100.0);
  EXPECT_FALSE(rectangle_in_rectangle.computeVisibilityPolygon(
      query, &visibility_polygon));
  //
  // expected_visibility_polygon = {
  //     Eigen::Vector2d(1.5, 0.5), Eigen::Vector2d(0.5, 0.5),
  //     Eigen::Vector2d(0.5, 1.5), Eigen::Vector2d(0.5, 2.0),
  //     Eigen::Vector2d(0.0, 2.0), Eigen::Vector2d(0.0, 0.0),
  //     Eigen::Vector2d(2.0, 0.0), Eigen::Vector2d(2.0, 0.5)};
  // removeRedundantVertices(&expected_visibility_polygon);
  //
  // EXPECT_EQ(expected_visibility_polygon.size(),
  //           visibility_polygon.getNumVertices());
  // for (size_t i = 0; i < visibility_polygon.getNumVertices(); i++) {
  //   EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(expected_visibility_polygon[i],
  //                                         visibility_polygon.getVertices()[i]))
  //       << "Expected: " << expected_visibility_polygon[i].transpose()
  //       << " Actual: " << visibility_polygon.getVertices()[i].transpose();
  // }
  //
  // query << -10.0, -10.0;
  // EXPECT_FALSE(
  //     poly_with_holes.computeVisibilityPolygon(query, &visibility_polygon));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
