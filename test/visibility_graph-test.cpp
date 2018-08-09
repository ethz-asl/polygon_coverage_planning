#include <eigen-checks/gtest.h>

#include "mav_coverage_planning/cost_function.h"
#include "mav_coverage_planning/graph/visibility_graph.h"

using namespace mav_coverage_planning;

TEST(VisibilityGraphTest, ShortestPath) {
  StdVector2d cw_polygon_vertices = {
      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(0.0, 2.0),
      Eigen::Vector2d(2.0, 2.0), Eigen::Vector2d(2.0, 0.0)};
  StdVector2d cc_hole_vertices = {
      Eigen::Vector2d(0.5, 0.5), Eigen::Vector2d(1.5, 0.5),
      Eigen::Vector2d(1.5, 1.5), Eigen::Vector2d(0.5, 1.5)};
  std::vector<StdVector2d> holes = {cc_hole_vertices};
  Polygon p(cw_polygon_vertices, holes);

  EXPECT_FALSE(p.isConvex());
  EXPECT_TRUE(p.isClockwise());
  EXPECT_TRUE(p.hasHoles());
  EXPECT_TRUE(p.hasSimpleHoles());
  EXPECT_FALSE(p.getHoles()[0].isClockwise());
  EXPECT_EQ(p.getNumHoles(), holes.size());
  EXPECT_EQ(p.getNumVertices(), cw_polygon_vertices.size());

  Eigen::Vector2d start, goal;
  start << -1.0, 0.05;
  EXPECT_FALSE(p.checkPointInPolygon(start));
  goal << 3.0, 3.0;
  EXPECT_FALSE(p.checkPointInPolygon(goal));
  EXPECT_FALSE(p.checkPointInPolygon(Eigen::Vector2d(1.0, 1.0)));

  Eigen::Vector2d start_new = p.projectPointOnHull(start);
  Eigen::Vector2d goal_new = p.projectPointOnHull(goal);
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(Eigen::Vector2d(0.0, 0.05), start_new));
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL(Eigen::Vector2d(2.0, 2.0), goal_new));

  CostFunction c;
  visibility_graph::VisibilityGraph graph(p, c);
  StdVector2d path;
  EXPECT_TRUE(graph.solve(start, goal, &path));

  StdVector2d expected_path = {start_new, Eigen::Vector2d(0.5, 1.5), goal_new};
  EXPECT_EQ(expected_path.size(), path.size());
  for (size_t i = 0; i < path.size(); i++) {
    EXPECT_TRUE(EIGEN_MATRIX_EQUAL(expected_path[i], path[i]));
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
