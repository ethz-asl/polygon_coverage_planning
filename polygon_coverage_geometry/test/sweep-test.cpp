/*
 * polygon_coverage_planning implements algorithms for coverage planning in
 * general polygons with holes. Copyright (C) 2019, Rik Bähnemann, Autonomous
 * Systems Lab, ETH Zürich
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

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
