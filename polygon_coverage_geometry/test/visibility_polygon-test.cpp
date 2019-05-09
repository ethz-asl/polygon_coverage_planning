#include <CGAL/is_y_monotone_2.h>
#include <gtest/gtest.h>

#include "polygon_coverage_geometry/cgal_comm.h"
#include "polygon_coverage_geometry/test_comm.h"
#include "polygon_coverage_geometry/visibility_polygon.h"

using namespace polygon_coverage_planning;

TEST(VisibilityPolygonTest, computeVisibilityPolygon) {
  PolygonWithHoles rectangle_in_rectangle(
      createRectangleInRectangle<Polygon_2, PolygonWithHoles>());
  Polygon_2 visibility_polygon;

  // Query on polygon vertex.
  Point_2 query(0.0, 0.0);
  EXPECT_TRUE(computeVisibilityPolygon(rectangle_in_rectangle, query,
                                       &visibility_polygon));
  // Result manually checked.
  VertexConstIterator vit = visibility_polygon.vertices_begin();
  EXPECT_EQ(9, visibility_polygon.size());
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
  EXPECT_TRUE(computeVisibilityPolygon(rectangle_in_rectangle, query,
                                       &visibility_polygon));
  vit = visibility_polygon.vertices_begin();
  EXPECT_EQ(6, visibility_polygon.size())
      << "Query point: " << query << " PWH: " << rectangle_in_rectangle
      << "Visibility: " << visibility_polygon;
  EXPECT_EQ(Point_2(1, 1.25), *vit++);
  EXPECT_EQ(Point_2(0, 1.25), *vit++);
  EXPECT_EQ(Point_2(0, 0), *vit++);
  EXPECT_EQ(Point_2(2, 0), *vit++);
  EXPECT_EQ(Point_2(2, 2), *vit++);
  EXPECT_EQ(Point_2(1, 2), *vit++);

  // Query in face.
  query = Point_2(1.0, 0.5);
  EXPECT_TRUE(computeVisibilityPolygon(rectangle_in_rectangle, query,
                                       &visibility_polygon));
  // Result manually checked.
  vit = visibility_polygon.vertices_begin();
  EXPECT_EQ(7, visibility_polygon.size());
  EXPECT_EQ(Point_2(1, 1.25), *vit++);
  EXPECT_EQ(Point_2(0.5, 1.25), *vit++);
  EXPECT_EQ(Point_2(0, 2), *vit++);
  EXPECT_EQ(Point_2(0, 0), *vit++);
  EXPECT_EQ(Point_2(2, 0), *vit++);
  EXPECT_EQ(Point_2(2, 2), *vit++);
  EXPECT_EQ(Point_2(1, 2), *vit++);

  // Query on polygon halfedge.
  query = Point_2(1.0, 0.0);
  EXPECT_TRUE(computeVisibilityPolygon(rectangle_in_rectangle, query,
                                       &visibility_polygon));
  // Result manually checked.
  vit = visibility_polygon.vertices_begin();
  EXPECT_EQ(8, visibility_polygon.size());
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
  EXPECT_TRUE(computeVisibilityPolygon(rectangle_in_rectangle, query,
                                       &visibility_polygon));
  // Result manually checked.
  vit = visibility_polygon.vertices_begin();
  EXPECT_EQ(4, visibility_polygon.size());
  EXPECT_EQ(Point_2(2, 1.25), *vit++);
  EXPECT_EQ(Point_2(0, 1.25), *vit++);
  EXPECT_EQ(Point_2(0, 0), *vit++);
  EXPECT_EQ(Point_2(2, 0), *vit++);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
