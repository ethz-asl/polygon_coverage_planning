#include <cstdlib>

#include <gtest/gtest.h>

#include "mav_2d_coverage_planning/cost_functions/euclidean_cost_function.h"
#include "mav_2d_coverage_planning/planners/polygon_stripmap_planner.h"
#include "mav_2d_coverage_planning/planners/polygon_stripmap_planner_exact.h"
#include "mav_2d_coverage_planning/planners/polygon_stripmap_planner_exact_preprocessed.h"
#include "mav_2d_coverage_planning/polygon.h"
#include "mav_2d_coverage_planning/tests/test_helpers.h"

using namespace mav_coverage_planning;

const double kCenterMin = -50.0;
const double kCenterMax = 50.0;
const double kPolygonDiameterMin = 20.0;
const double kPolygonDiameterMax = 100.0;
const double kAltitudeMin = 0.5;
const double kAltitudeMax = 30.0;
const double kFOVCameraRadMin = M_PI / 12.0;
const double kFOVCameraRadMax = M_PI - 0.1;
const double kMinViewOverlapMin = 0.0;
const double kMinViewOverlapMax = 0.99;
const int kNumPolygons = 1e2;
const size_t kSeed = 123456;
const double kNear = 1e-3;

// Given a set of polygons run all planners.
void runPlanners(const std::vector<Polygon>& polygons) {
  for (const Polygon& p : polygons) {
    // Create planner settings.
    PolygonStripmapPlanner::Settings settings;
    settings.polygon = p;
    EXPECT_EQ(0, settings.polygon.getPolygon().number_of_holes());

    settings.segment_cost_function =
        std::bind(&computeEuclideanSegmentCost, std::placeholders::_1,
                  std::placeholders::_2);
    settings.path_cost_function =
        std::bind(&computeEuclideanPathCost, std::placeholders::_1);
    settings.altitude = createRandomDouble(kAltitudeMin, kAltitudeMax);
    settings.lateral_fov =
        createRandomDouble(kFOVCameraRadMin, kFOVCameraRadMax);
    settings.longitudinal_fov =
        createRandomDouble(kFOVCameraRadMin, kFOVCameraRadMax);
    settings.min_view_overlap =
        createRandomDouble(kMinViewOverlapMin, kMinViewOverlapMax);
    EXPECT_TRUE(settings.check());

    // Create planners.
    PolygonStripmapPlanner planner_gk_ma(settings);
    PolygonStripmapPlannerExact planner_exact(settings);
    PolygonStripmapPlannerExactPreprocessed planner_exact_preprocessed(
        settings);

    EXPECT_TRUE(planner_gk_ma.setup());
    EXPECT_TRUE(planner_exact.setup());
    EXPECT_TRUE(planner_exact_preprocessed.setup());
    EXPECT_TRUE(planner_gk_ma.isInitialized());
    EXPECT_TRUE(planner_exact.isInitialized());
    EXPECT_TRUE(planner_exact_preprocessed.isInitialized());

    std::vector<Point_2> waypoints_gk_ma, waypoints_exact,
        waypoints_exact_preprocessed;
    Point_2 start = Point_2(CGAL::ORIGIN);
    Point_2 goal = Point_2(CGAL::ORIGIN);

    EXPECT_TRUE(planner_gk_ma.solve(start, goal, &waypoints_gk_ma));
    EXPECT_TRUE(planner_exact.solve(start, goal, &waypoints_exact));
    EXPECT_TRUE(planner_exact_preprocessed.solve(
        start, goal, &waypoints_exact_preprocessed));

    EXPECT_LT(2, waypoints_gk_ma.size());
    EXPECT_LT(2, waypoints_exact.size());
    EXPECT_LT(2, waypoints_exact_preprocessed.size());

    // Start and goal may lie outside of polygon.
    EXPECT_TRUE(settings.polygon.pointsInPolygon(
        std::next(waypoints_gk_ma.begin()), std::prev(waypoints_gk_ma.end())));
    EXPECT_TRUE(settings.polygon.pointsInPolygon(
        std::next(waypoints_exact.begin()), std::prev(waypoints_exact.end())));
    EXPECT_TRUE(settings.polygon.pointsInPolygon(
        std::next(waypoints_exact_preprocessed.begin()),
        std::prev(waypoints_exact_preprocessed.end())));

    EXPECT_EQ(settings.path_cost_function(waypoints_exact),
              settings.path_cost_function(waypoints_exact_preprocessed));
    EXPECT_NEAR(settings.path_cost_function(waypoints_gk_ma),
                settings.path_cost_function(waypoints_exact), kNear);
  }
}

TEST(StripmapPlannerTest, RandomConvexPolygon) {
  std::srand(kSeed);
  std::vector<Polygon> polygons(kNumPolygons);

  for (size_t i = 0; i < kNumPolygons; i++) {
    double x_0 = createRandomDouble(kCenterMin, kCenterMax);
    double y_0 = createRandomDouble(kCenterMin, kCenterMax);
    double r =
        createRandomDouble(kPolygonDiameterMin, kPolygonDiameterMax) / 2.0;
    polygons[i] = Polygon(createRandomConvexPolygon<Polygon_2, K>(x_0, y_0, r));
    if (polygons[i].getPolygon().outer_boundary().size() <= 2) continue;
    EXPECT_TRUE(polygons[i].isConvex());
  }
  runPlanners(polygons);
}

TEST(StripmapPlannerTest, RandomSimplePolygon) {
  CGAL::Random random(kSeed);
  std::srand(kSeed);
  std::vector<Polygon> polygons(kNumPolygons);

  for (size_t i = 0; i < kNumPolygons; i++) {
    double r =
        createRandomDouble(kPolygonDiameterMin, kPolygonDiameterMax) / 2.0;
    const int kMaxPolySize = 10;
    polygons[i] = Polygon(
        createRandomSimplePolygon<Polygon_2, K>(r, random, kMaxPolySize));
    if (polygons[i].getPolygon().outer_boundary().size() <= 2) continue;
    EXPECT_TRUE(polygons[i].isStrictlySimple());
  }
  runPlanners(polygons);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  return RUN_ALL_TESTS();
}
