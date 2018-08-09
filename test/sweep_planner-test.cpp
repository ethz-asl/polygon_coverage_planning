#include <cstdlib>

#include <eigen-checks/gtest.h>

#include "mav_coverage_planning/cost_function.h"
#include "mav_coverage_planning/math.h"
#include "mav_coverage_planning/polygon.h"
#include "mav_coverage_planning/sweep_planner.h"

using namespace mav_coverage_planning;

double createRandomDouble(double min, double max) {
  // No seed for repeatability.
  return (max - min) * (static_cast<double>(std::rand()) /
                        static_cast<double>(RAND_MAX)) +
         min;
}

TEST(LineSweepPlannerTest, ConvexPolygon) {
  const double kCenterMin = -50.0;
  const double kCenterMax = 50.0;
  const double kPolygonDiameterMin = 1.0;
  const double kPolygonDiameterMax = 100.0;
  const double kAltitudeMin = 0.5;
  const double kAltitudeMax = 30.0;
  const double kFOVCameraRadMin = M_PI / 12.0;
  const double kFOVCameraRadMax = M_PI - 0.1;
  const double kMinViewOverlapMin = 0.0;
  const double kMinViewOverlapMax = 0.99;

  const int kNumPaths = 1.0e0;
  std::srand(1);
  for (size_t i = 0; i < kNumPaths; i++) {
    double x_0 = createRandomDouble(kCenterMin, kCenterMax);
    double y_0 = createRandomDouble(kCenterMin, kCenterMax);
    double r =
        createRandomDouble(kPolygonDiameterMin, kPolygonDiameterMax) / 2.0;
    StdVector2d cw_polygon_vertices;
    if (!createRandomConvexPolygon(x_0, y_0, r, &cw_polygon_vertices)) {
      std::cout << "Only 2 vertices polygon." << std::endl;
      continue;
    }
    Polygon p(cw_polygon_vertices);
    EXPECT_TRUE(p.isValid());
    EXPECT_TRUE(p.isClockwise());
    EXPECT_TRUE(p.isConvex());
    EXPECT_FALSE(p.hasHoles());
    EXPECT_FALSE(p.hasSimpleHoles());

    double altitude = createRandomDouble(kAltitudeMin, kAltitudeMax);
    double fov_camera_rad =
        createRandomDouble(kFOVCameraRadMin, kFOVCameraRadMax);
    double min_view_overlap =
        createRandomDouble(kMinViewOverlapMin, kMinViewOverlapMax);
    CostFunction c;
    SweepPlanner planner(p, c, altitude, fov_camera_rad, min_view_overlap,
                         GTSPPSolver::kExactWithPreprocessing);
    EXPECT_TRUE(planner.isInitialized());
    StdVector2d waypoints;
    Eigen::Vector2d start = Eigen::Vector2d::Zero();
    Eigen::Vector2d goal = Eigen::Vector2d::Zero();
    EXPECT_TRUE(planner.solve(start, goal, &waypoints));

    EXPECT_FALSE(waypoints.empty());
    for (size_t i = 0; i < waypoints.size(); i++) {
      if (i == 0 || i == waypoints.size() - 1) {
        continue;  // Start and goal may lie outside of polygon.
      }
      EXPECT_TRUE(p.checkPointInPolygon(waypoints[i]))
          << waypoints[i].transpose();
    }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
