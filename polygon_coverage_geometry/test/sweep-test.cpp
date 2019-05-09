#include <CGAL/is_y_monotone_2.h>
#include <gtest/gtest.h>

#include "polygon_coverage_geometry/cgal_comm.h"
#include "polygon_coverage_geometry/sweep.h"
#include "polygon_coverage_geometry/test_comm.h"

using namespace polygon_coverage_planning;

TEST(SweepTest, computeAllSweeps) {
  const double kMaxSweepDistance = 0.1;

  Polygon_2 diamond(createDiamond<Polygon_2>());
  std::vector<std::vector<Point_2>> cluster_sweeps;
  EXPECT_TRUE(computeAllSweeps(diamond, kMaxSweepDistance, &cluster_sweeps));
  EXPECT_EQ(cluster_sweeps.size(), 8) << diamond;
  for (const std::vector<Point_2>& waypoints : cluster_sweeps) {
    EXPECT_GE(waypoints.size(), 4);
    for (const Point_2& p : waypoints) EXPECT_TRUE(pointInPolygon(diamond, p));
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
