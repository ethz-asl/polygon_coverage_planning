#include <cstdlib>

#include <eigen-checks/gtest.h>

#include "mav_coverage_planning/cgal.h"
#include "mav_coverage_planning/common.h"
#include "mav_coverage_planning/math.h"

using namespace mav_coverage_planning;

bool numberInClosedInterval(double number, double min, double max) {
  if ((number > min) && (number < max)) {
    return true;
  } else {
    return false;
  }
}

double createRandomDouble(double min, double max) {
  return (max - min) * (static_cast<double>(std::rand()) /
                        static_cast<double>(RAND_MAX)) +
         min;
}

TEST(MathTest, RayLineIntersection) {
  // Simple known cases.
  // Simple perpendicular intersection.
  Eigen::Vector2d ray_origin(0.0, 0.0);
  Eigen::Vector2d ray_direction(1.0, 0.0);
  Eigen::Vector2d line_start(1.0, 1.0);
  Eigen::Vector2d line_end(1.0, -1.0);
  double expected_t_1 = 1.0;
  double computed_t_1;
  bool is_ray_intersecting_line = computeRayLineIntersection(
      ray_origin, ray_direction, line_start, line_end, &computed_t_1);
  EXPECT_TRUE(is_ray_intersecting_line);
  EXPECT_EQ(expected_t_1, computed_t_1);

  // Parallel non-intercepting.
  line_end << 2.0, 1.0;
  expected_t_1 = -1.0;
  is_ray_intersecting_line = computeRayLineIntersection(
      ray_origin, ray_direction, line_start, line_end, &computed_t_1);
  EXPECT_FALSE(is_ray_intersecting_line);
  EXPECT_EQ(expected_t_1, computed_t_1);

  // Collinear overlapping.
  line_start << -1.0, 0.0;
  line_end << 1.0, 0.0;
  expected_t_1 = 0.0;
  is_ray_intersecting_line = computeRayLineIntersection(
      ray_origin, ray_direction, line_start, line_end, &computed_t_1);
  EXPECT_TRUE(is_ray_intersecting_line);
  EXPECT_EQ(expected_t_1, computed_t_1);

  // Collinear ray starting outside, intercepting.
  ray_origin << -2.0, 0.0;
  expected_t_1 = 1.0;
  is_ray_intersecting_line = computeRayLineIntersection(
      ray_origin, ray_direction, line_start, line_end, &computed_t_1);
  EXPECT_TRUE(is_ray_intersecting_line);
  EXPECT_EQ(expected_t_1, computed_t_1);

  // Collinear ray starting outside, non-intercepting.
  ray_origin << 2.0, 0.0;
  expected_t_1 = -1.0;
  is_ray_intersecting_line = computeRayLineIntersection(
      ray_origin, ray_direction, line_start, line_end, &computed_t_1);
  EXPECT_FALSE(is_ray_intersecting_line);
  EXPECT_EQ(expected_t_1, computed_t_1);

  // Ray passing line.
  ray_origin << 0.0, 0.0;
  line_start << 1.0, 5.0;
  line_end << 1.0, 4.0;
  expected_t_1 = 1.0;
  is_ray_intersecting_line = computeRayLineIntersection(
      ray_origin, ray_direction, line_start, line_end, &computed_t_1);
  EXPECT_FALSE(is_ray_intersecting_line);
  EXPECT_EQ(expected_t_1, computed_t_1);

  // All start in the same point.
  line_start << 0.0, 0.0;
  line_end << 0.0, 0.0;
  ray_origin << 0.0, 0.0;
  expected_t_1 = 0.0;
  is_ray_intersecting_line = computeRayLineIntersection(
      ray_origin, ray_direction, line_start, line_end, &computed_t_1);
  EXPECT_TRUE(is_ray_intersecting_line);
  EXPECT_EQ(expected_t_1, computed_t_1);
}

TEST(MathTest, InwardFacingNormals) {
  // Square.
  StdVector2d square, normals, expected_normals;
  square.push_back(Eigen::Vector2d(0.0, 0.0));
  square.push_back(Eigen::Vector2d(0.0, 2.0));
  square.push_back(Eigen::Vector2d(2.0, 2.0));
  square.push_back(Eigen::Vector2d(2.0, 0.0));

  computeInwardFacingNormals(square, &normals);

  EXPECT_EQ(square.size(), normals.size());

  expected_normals.push_back(Eigen::Vector2d(1.0, 0.0));
  expected_normals.push_back(Eigen::Vector2d(0.0, -1.0));
  expected_normals.push_back(Eigen::Vector2d(-1.0, 0.0));
  expected_normals.push_back(Eigen::Vector2d(0.0, 1.0));

  for (size_t i = 0; i < normals.size(); i++) {
    EXPECT_TRUE(EIGEN_MATRIX_EQUAL(expected_normals[i], normals[i]));
  }
}

TEST(MathTest, PointInConvexPolygon) {
  // Square.
  StdVector2d square, normals;
  double x_min = 0.0;
  double x_max = 2.0;
  double y_min = 0.0;
  double y_max = 2.0;
  square.push_back(Eigen::Vector2d(x_min, y_min));
  square.push_back(Eigen::Vector2d(x_min, y_max));
  square.push_back(Eigen::Vector2d(x_max, y_max));
  square.push_back(Eigen::Vector2d(x_max, y_min));
  computeInwardFacingNormals(square, &normals);

  size_t num_checks = 1000;
  double sampling_bounds = 1.0;
  std::srand(123456);
  for (size_t i = 0; i < num_checks; i++) {
    Eigen::Vector2d p(createRandomDouble(-sampling_bounds, sampling_bounds),
                      createRandomDouble(-sampling_bounds, sampling_bounds));
    bool expected_outcome = (numberInClosedInterval(p.x(), x_min, x_max) &&
                             numberInClosedInterval(p.y(), y_min, y_max));
    bool outcome = pointInConvexPolygon(p, square, normals);
    EXPECT_EQ(expected_outcome, outcome);
  }
}

TEST(MathTest, PointInComplexPolygon) {
  // Square.
  StdVector2d square;
  double x_min = 0.0;
  double x_max = 2.0;
  double y_min = 0.0;
  double y_max = 2.0;
  square.push_back(Eigen::Vector2d(x_min, y_min));
  square.push_back(Eigen::Vector2d(x_min, y_max));
  square.push_back(Eigen::Vector2d(x_max, y_max));
  square.push_back(Eigen::Vector2d(x_max, y_min));

  size_t num_checks = 1000;
  std::srand(654321);
  double sampling_bounds = 1.0;
  for (size_t i = 0; i < num_checks; i++) {
    Eigen::Vector2d p(createRandomDouble(-sampling_bounds, sampling_bounds),
                      createRandomDouble(-sampling_bounds, sampling_bounds));
    bool expected_outcome = (numberInClosedInterval(p.x(), x_min, x_max) &&
                             numberInClosedInterval(p.y(), y_min, y_max));
    bool outcome = pointInPolygonWindingNumber(p, square);
    EXPECT_EQ(expected_outcome, outcome) << "Point: " << p.transpose();
  }

  // On hull.
  Eigen::Vector2d p(0.0, 0.0);
  EXPECT_TRUE(pointInPolygonWindingNumber(p, square))
      << "Point: " << p.transpose();
  p << 0.5, 0.0;
  EXPECT_TRUE(pointInPolygonWindingNumber(p, square))
      << "Point: " << p.transpose();

  // Example non-convex polygon.
  StdVector2d polygon = {Eigen::Vector2d(1.5, 0.5), Eigen::Vector2d(0.0, 0.5),
                         Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(2.0, 0.0),
                         Eigen::Vector2d(2.0, 2.0), Eigen::Vector2d(1.5, 2.0)};
  p << 0.3, 0.3;
  EXPECT_TRUE(pointInPolygonWindingNumber(p, polygon))
      << "Point: " << p.transpose();

  p << 0.5, 0.5;  //-std::numeric_limits<double>::epsilon();
  EXPECT_TRUE(pointInPolygonWindingNumber(p, polygon))
      << "Point: " << p.transpose();
}

TEST(MathTest, ShrinkPolygon) {
  // Square.
  StdVector2d square, expected_small_square;
  square.push_back(Eigen::Vector2d(0.0, 0.0));
  square.push_back(Eigen::Vector2d(0.0, 2.0));
  square.push_back(Eigen::Vector2d(2.0, 2.0));
  square.push_back(Eigen::Vector2d(2.0, 0.0));

  expected_small_square.push_back(Eigen::Vector2d(0.5, 0.5));
  expected_small_square.push_back(Eigen::Vector2d(0.5, 1.5));
  expected_small_square.push_back(Eigen::Vector2d(1.5, 1.5));
  expected_small_square.push_back(Eigen::Vector2d(1.5, 0.5));

  double distance = std::sqrt(0.5);
  StdVector2d computed_small_square;
  bool is_successful = shrinkPolygon(square, distance, &computed_small_square);
  EXPECT_TRUE(is_successful);
  EXPECT_EQ(expected_small_square.size(), computed_small_square.size());
  for (size_t i = 0; i < computed_small_square.size(); i++) {
    EXPECT_TRUE(
        EIGEN_MATRIX_EQUAL(expected_small_square[i], computed_small_square[i]));
  }
}

TEST(MathTest, FurthestVertex) {
  Eigen::Vector2d v1, v2, v3, v4, n;
  v1 << 0.0, 0.0;
  v2 << 1.0, 0.0;
  v3 << 10.0, -2142.2;
  v4 << -2.0, -13;
  n << 1.0, 0.0;

  StdVector2d vertices;
  vertices.push_back(v1);
  vertices.push_back(v2);
  vertices.push_back(v3);
  vertices.push_back(v4);

  std::vector<double> distances;
  getProjectedVertexDistances(vertices, v1, n, &distances);
  EXPECT_EQ(v1.x(), distances[0]);
  EXPECT_EQ(v2.x(), distances[1]);
  EXPECT_EQ(v3.x(), distances[2]);
  EXPECT_EQ(v4.x(), distances[3]);
}

TEST(MathTest, PointLineDistance) {
  Eigen::Vector2d point(5.0, 2.0);
  Eigen::Vector2d line_start(-1.0, 0.0);
  Eigen::Vector2d line_end(1.0, 0.0);
  double computed_distance;
  computeDistancePointToLine(point, line_start, line_end, &computed_distance);
  double expected_distance = 2.0;
  EXPECT_EQ(expected_distance, computed_distance);
}

TEST(MathTest, ProjectLineOnPolygon) {
  // Square.
  StdVector2d square;
  square.push_back(Eigen::Vector2d(0.0, 0.0));
  square.push_back(Eigen::Vector2d(0.0, 2.0));
  square.push_back(Eigen::Vector2d(2.0, 2.0));
  square.push_back(Eigen::Vector2d(2.0, 0.0));

  // Test 1.
  // Line
  Eigen::Vector2d line_start(0.0, 1.0);
  Eigen::Vector2d line_end(1.0, 2.0);
  Eigen::Vector2d normal = computeLeftFacingNormal(line_end - line_start);
  // Projection
  StdVector2d computed_projection;
  EXPECT_TRUE(castPerpendicularShadowOnVertices(square, line_start, line_end,
                                                normal, &computed_projection));
  StdVector2d expected_projection = {Eigen::Vector2d(0.0, 1.0),
                                     Eigen::Vector2d(0.0, 2.0),
                                     Eigen::Vector2d(1.0, 2.0)};
  EXPECT_EQ(expected_projection.size(), computed_projection.size());
  for (size_t i = 0; i < expected_projection.size(); i++) {
    EXPECT_TRUE(
        EIGEN_MATRIX_EQUAL(expected_projection[i], computed_projection[i]));
  }

  // Test 2.
  line_start << 0, 0.5;
  line_end << 0, 1.5;
  normal = computeLeftFacingNormal(line_end - line_start);
  expected_projection = {line_start, line_end};
  EXPECT_TRUE(castPerpendicularShadowOnVertices(square, line_start, line_end,
                                                normal, &computed_projection));
  EXPECT_EQ(expected_projection.size(), computed_projection.size());
  for (size_t i = 0; i < expected_projection.size(); i++) {
    EXPECT_TRUE(
        EIGEN_MATRIX_EQUAL(expected_projection[i], computed_projection[i]));
  }
}

TEST(MathTest, PolygonValid) {
  // Convex.
  StdVector2d square;
  square.push_back(Eigen::Vector2d(0.0, 0.0));
  square.push_back(Eigen::Vector2d(0.0, 2.0));
  square.push_back(Eigen::Vector2d(2.0, 2.0));
  square.push_back(Eigen::Vector2d(2.0, 0.0));
  EXPECT_TRUE(isPolygonValid(square));
  std::reverse(square.begin(), square.end());
  EXPECT_FALSE(isPolygonValid(square));
}

TEST(MathTest, CorrectVertices) {
  Eigen::Vector2d v1(0.0, 0.0);
  Eigen::Vector2d v2(1.0, 3.0);
  StdVector2d vertices = {v2, v1, v2, v2, v2};
  correctVertices(&vertices);
  EXPECT_EQ(2, vertices.size());
}

TEST(MathTest, SimplePolygonTest) {
  // TODO(rikba): Create more random polygons with hole.
  StdVector2d cw_polygon_vertices = {
      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, 2.0),
      Eigen::Vector2d(2.0, 2.0), Eigen::Vector2d(2.0, 0.0)};
  StdVector2d cc_hole_vertices = {
      Eigen::Vector2d(1.0, 1.25), Eigen::Vector2d(1.0, 1.75),
      Eigen::Vector2d(0.5, 1.75), Eigen::Vector2d(0.5, 1.25)};
  std::vector<StdVector2d> holes = {cc_hole_vertices};

  StdVector2d simple_polygon;
  polygonWithHolesToSimplePolygon(cw_polygon_vertices, holes, &simple_polygon);

  EXPECT_EQ(
      cc_hole_vertices.size() + cc_hole_vertices.size() + 2 * holes.size(),
      simple_polygon.size());

  cgal::PartitioningPolygon cw_simple_polygon;
  cgal::eigenVerticesToPartitioningPolygon(simple_polygon, &cw_simple_polygon);
  EXPECT_TRUE(cw_simple_polygon.is_clockwise_oriented());
  EXPECT_TRUE(cw_simple_polygon.is_simple());
}

TEST(MathTest, KElementsFromNTest) {
  std::vector<size_t> sorted_elements = {0, 1, 2};
  std::vector<std::set<size_t>> combinations;

  for (size_t k = 0; k <= sorted_elements.size(); k++) {
    getAllCombinationsOfKElementsFromN(sorted_elements, k, &combinations);
    EXPECT_EQ(combinations.size(), nChooseK(sorted_elements.size(), k));
    for (const std::set<size_t>& combination : combinations) {
      EXPECT_EQ(combination.size(), k);
    }
    if (k == 1) {
      EXPECT_NE(combinations[0].find(0), combinations[0].end());
      EXPECT_NE(combinations[1].find(1), combinations[1].end());
      EXPECT_NE(combinations[2].find(2), combinations[2].end());
    } else if (k == 2) {
      EXPECT_NE(combinations[0].find(0), combinations[0].end());
      EXPECT_NE(combinations[0].find(1), combinations[0].end());

      EXPECT_NE(combinations[1].find(0), combinations[0].end());
      EXPECT_NE(combinations[1].find(2), combinations[0].end());

      EXPECT_NE(combinations[2].find(1), combinations[0].end());
      EXPECT_NE(combinations[2].find(2), combinations[0].end());
    } else if (k == 3) {
      EXPECT_NE(combinations[0].find(0), combinations[0].end());
      EXPECT_NE(combinations[0].find(1), combinations[0].end());
      EXPECT_NE(combinations[0].find(2), combinations[0].end());
    }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
