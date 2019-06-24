#include <functional>

#include <gtest/gtest.h>

#include "polygon_coverage_geometry/cgal_comm.h"
#include "polygon_coverage_geometry/test_comm.h"
#include "polygon_coverage_geometry/visibility_graph.h"

using namespace polygon_coverage_planning;

TEST(VisibilityGraphTest, ShortestPath) {
  PolygonWithHoles p(createRectangleInRectangle<Polygon_2, PolygonWithHoles>());

  Point_2 start(-1.0, 3.0);
  EXPECT_FALSE(pointInPolygon(p, start));
  Point_2 goal(3.0, -1.0);
  EXPECT_FALSE(pointInPolygon(p, goal));

  visibility_graph::VisibilityGraph graph(p);
  EXPECT_TRUE(graph.isInitialized());
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
