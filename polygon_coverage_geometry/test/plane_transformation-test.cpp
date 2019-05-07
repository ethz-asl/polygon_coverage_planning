#include <fstream>
#include <string>

#include <CGAL/Random.h>
#include <gtest/gtest.h>

#include "mav_2d_coverage_planning/geometry/plane_transformation.h"
#include "mav_2d_coverage_planning/tests/test_helpers.h"
#include "mav_coverage_planning_comm/cgal_definitions.h"

using namespace mav_coverage_planning;

const size_t kNumRuns = 1e3;
const double kHalfCubeSide = 100.0;
const double kPolygonRadius = 100.0;
const int kMaxPolySize = 10;
const size_t kSeed = 123456;
const double kPrecision = 1e-9;

TEST(MeshProcessing, PlaneTransformation) {
  // Plane parallel to xy plane.
  for (FT d = -5.0; d < 6.0; d = d + 1.0) {
    Plane_3 plane(0.0, 0.0, 1.0, d);
    PlaneTransformation<K> plane_tf(plane);
    Point_3 p_3 = plane.point();
    Point_2 p_2 = plane_tf.to2d(p_3);
    EXPECT_EQ(p_2.x(), p_3.x());
    EXPECT_EQ(p_2.y(), p_3.y());

    Point_3 p_3_back = plane_tf.to3d(p_2);
    EXPECT_EQ(p_3, p_3_back);
  }

  CGAL::Random random(kSeed);
  size_t i = 0;
  while (i++ < kNumRuns) {
    // Create random plane.
    Plane_3 plane = createRandomPlane<K>(kHalfCubeSide, random);
    PlaneTransformation<K> plane_tf(plane);
    // Create random 2D polygon.
    Polygon_2 polygon = createRandomSimplePolygon<Polygon_2, K>(
        kPolygonRadius, random, kMaxPolySize);
    Polygon_2::Edge_const_circulator eit = polygon.edges_circulator();
    for (Polygon_2::Vertex_const_iterator it = polygon.vertices_begin();
         it < polygon.vertices_end(); ++it) {
      Point_3 p3 = plane_tf.to3d(*it);
      EXPECT_NEAR(std::sqrt(CGAL::to_double(CGAL::squared_distance(p3, plane))),
                  0.0, kPrecision);
      Point_3 p3_on_plane = plane_tf.to3dOnPlane(*it);
      EXPECT_TRUE(plane.has_on(p3_on_plane));
      Point_2 p2 = plane_tf.to2d(p3);
      EXPECT_EQ(*it, p2);
      Point_2 p2_on_plane = plane_tf.to2d(p3_on_plane);
      EXPECT_NEAR(CGAL::to_double(it->x()), CGAL::to_double(p2_on_plane.x()),
                  kPrecision);
      EXPECT_NEAR(CGAL::to_double(it->y()), CGAL::to_double(p2_on_plane.y()),
                  kPrecision);
    }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
