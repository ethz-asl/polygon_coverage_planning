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

#include <cstdlib>

#include <CGAL/Random.h>
#include <gtest/gtest.h>
#include <ros/console.h>

#include <polygon_coverage_geometry/cgal_comm.h>
#include <polygon_coverage_geometry/test_comm.h>

#include "polygon_coverage_planners/cost_functions/path_cost_functions.h"
#include "polygon_coverage_planners/graphs/sweep_plan_graph.h"
#include "polygon_coverage_planners/planners/polygon_stripmap_planner.h"
#include "polygon_coverage_planners/planners/polygon_stripmap_planner_exact.h"
#include "polygon_coverage_planners/planners/polygon_stripmap_planner_exact_preprocessed.h"
#include "polygon_coverage_planners/sensor_models/frustum.h"

using namespace polygon_coverage_planning;

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
void runPlanners(const std::vector<PolygonWithHoles>& polygons) {
  for (const PolygonWithHoles& p : polygons) {
    // Create planner settings.
    sweep_plan_graph::SweepPlanGraph::Settings settings;
    settings.polygon = p;
    ROS_INFO_STREAM(p);
    settings.cost_function =
        std::bind(&computeEuclideanPathCost, std::placeholders::_1);
    settings.sensor_model = std::make_shared<Frustum>(
        createRandomDouble(kAltitudeMin, kAltitudeMax),
        createRandomDouble(kFOVCameraRadMin, kFOVCameraRadMax),
        createRandomDouble(kMinViewOverlapMin, kMinViewOverlapMax));
    settings.decomposition_type = DecompositionType::kBCD;
    EXPECT_EQ(static_cast<size_t>(0), settings.polygon.number_of_holes());

    settings.offset_polygons = false;

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

    EXPECT_LT(static_cast<size_t>(2), waypoints_gk_ma.size());
    EXPECT_LT(static_cast<size_t>(2), waypoints_exact.size());
    EXPECT_LT(static_cast<size_t>(2), waypoints_exact_preprocessed.size());

    // Start and goal may lie outside of polygon.
    EXPECT_TRUE(pointsInPolygon(settings.polygon,
                                std::next(waypoints_gk_ma.begin()),
                                std::prev(waypoints_gk_ma.end())));
    EXPECT_TRUE(pointsInPolygon(settings.polygon,
                                std::next(waypoints_exact.begin()),
                                std::prev(waypoints_exact.end())));
    EXPECT_TRUE(pointsInPolygon(settings.polygon,
                                std::next(waypoints_exact_preprocessed.begin()),
                                std::prev(waypoints_exact_preprocessed.end())));

    EXPECT_GT(settings.cost_function(waypoints_exact), 0.0);
    EXPECT_EQ(settings.cost_function(waypoints_exact),
              settings.cost_function(waypoints_exact_preprocessed));
    EXPECT_GE(settings.cost_function(waypoints_gk_ma) + kNear,
              settings.cost_function(waypoints_exact));
  }
}

TEST(StripmapPlannerTest, RandomConvexPolygon) {
  std::srand(kSeed);
  std::vector<PolygonWithHoles> polygons(kNumPolygons);

  for (size_t i = 0; i < kNumPolygons; i++) {
    double x_0 = createRandomDouble(kCenterMin, kCenterMax);
    double y_0 = createRandomDouble(kCenterMin, kCenterMax);
    double r =
        createRandomDouble(kPolygonDiameterMin, kPolygonDiameterMax) / 2.0;
    polygons[i] = createRandomConvexPolygon<PolygonWithHoles, K>(x_0, y_0, r);
    if (polygons[i].outer_boundary().size() <= 2) continue;
    EXPECT_TRUE(polygons[i].outer_boundary().is_simple());
    EXPECT_TRUE(polygons[i].outer_boundary().is_convex());
  }
  runPlanners(polygons);
}

TEST(StripmapPlannerTest, RandomSimplePolygon) {
  CGAL::Random random(kSeed);
  std::srand(kSeed);
  std::vector<PolygonWithHoles> polygons(kNumPolygons);

  for (size_t i = 0; i < kNumPolygons; i++) {
    double r =
        createRandomDouble(kPolygonDiameterMin, kPolygonDiameterMax) / 2.0;
    const int kMaxPolySize = 10;
    polygons[i] =
        createRandomSimplePolygon<PolygonWithHoles, K>(r, random, kMaxPolySize);
    if (polygons[i].outer_boundary().size() <= static_cast<size_t>(2)) continue;
    EXPECT_TRUE(isStrictlySimple(polygons[i]));
  }
  runPlanners(polygons);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
