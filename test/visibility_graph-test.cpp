#include <gtest/gtest.h>

#include "mav_2d_coverage_planning/graphs/visibility_graph.h"
#include "mav_2d_coverage_planning/polygon.h"
#include "mav_2d_coverage_planning/tests/test_helpers.h"

using namespace mav_coverage_planning;

TEST(VisibilityGraphTest, ShortestPath) {
  Polygon p = createRectangleInRectangle();

  Point_2 start(-1.0, 3.0);
  EXPECT_FALSE(p.pointInPolygon(start));
  Point_2 goal(3.0, -1.0);
  EXPECT_FALSE(p.pointInPolygon(goal));

  visibility_graph::VisibilityGraph graph(p);
  std::vector<Point_2> path;
  EXPECT_TRUE(graph.solve(start, goal, &path));

  std::vector<Point_2> expected_path = {Point_2(0, 2), Point_2(0.5, 1.25),
                                        Point_2(2, 0)};
  EXPECT_EQ(expected_path.size(), path.size());
  for (size_t i = 0; i < path.size(); i++) {
    EXPECT_EQ(expected_path[i], path[i]) << i;
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
