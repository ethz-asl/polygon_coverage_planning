#include <iomanip>
#include <sstream>

#include <CGAL/Boolean_set_operations_2.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <mav_planning_msgs/PolygonWithHoles.h>

#include <mav_2d_coverage_planning/cost_functions/path_cost_functions.h>
#include <mav_2d_coverage_planning/geometry/polygon.h>
#include <mav_2d_coverage_planning/planners/polygon_stripmap_planner.h>
#include <mav_2d_coverage_planning/planners/polygon_stripmap_planner_exact.h>
#include <mav_2d_coverage_planning/sensor_models/line.h>
#include <mav_coverage_planning_comm/cgal_definitions.h>
#include <mav_coverage_planning_comm/timing.h>

const std::string kPackageName = "mav_coverage_planning_ros";
const size_t kMaxNoObstacles = 10;
const size_t kNthObstacle = 5;
const size_t kObstacleBins = kMaxNoObstacles / kNthObstacle + 1;
const size_t kNoInstances = 100;
const double kSweepDistance = 3.0;
const double kOverlap = 0.0;
const double kVMax = 3.0;
const double kAMax = 1.0;
const Point_2 kStart(0.0, 0.0);
const Point_2 kGoal = kStart;
const double kMapScale = 0.025;

using namespace mav_coverage_planning;

bool loadPolygonFromNode(const YAML::Node& node, Polygon_2* poly) {
  CHECK_NOTNULL(poly);
  if (!node) return false;
  YAML::Node points = node["points"];
  if (!points) return false;
  if (points.size() < 3) return false;
  poly->clear();

  for (size_t i = 0; i < points.size(); ++i) {
    YAML::Node point = points[i];
    if (!point["x"]) return false;
    if (!point["y"]) return false;
    Point_2 p(kMapScale * point["x"].as<double>(),
              kMapScale * point["y"].as<double>());
    poly->push_back(p);
  }

  return true;
}

bool loadPWHFromFile(const std::string& file, Polygon* polygon) {
  CHECK_NOTNULL(polygon);

  YAML::Node node = YAML::LoadFile(file);

  PolygonWithHoles pwh;
  if (!loadPolygonFromNode(node["hull"], &pwh.outer_boundary())) return false;

  for (size_t i = 0; i < node["holes"].size(); ++i) {
    Polygon_2 poly;
    if (!loadPolygonFromNode(node["holes"][i], &poly)) return false;
    std::list<PolygonWithHoles> diff;
    CGAL::difference(pwh, poly, std::back_inserter(diff));
    pwh = *diff.begin();
  }

  CHECK_NOTNULL(polygon);
  *polygon = Polygon(pwh);

  return true;
}

size_t computeNoHoleVertices(const Polygon& poly) {
  size_t no_hole_vertices = 0;
  for (PolygonWithHoles::Hole_const_iterator hit =
           poly.getPolygon().holes_begin();
       hit != poly.getPolygon().holes_end(); ++hit) {
    no_hole_vertices += hit->size();
  }
  return no_hole_vertices;
}

bool loadAllInstances(std::vector<std::vector<Polygon>>* polys) {
  CHECK_NOTNULL(polys);
  polys->resize(kObstacleBins, std::vector<Polygon>(kNoInstances));

  std::string instances_path = ros::package::getPath(kPackageName);
  instances_path = instances_path.substr(0, instances_path.find("/src/"));
  instances_path +=
      "/build/" + kPackageName + "/pwh_instances-prefix/src/pwh_instances/";

  for (size_t i = 0; i < kObstacleBins; i++) {
    std::string subfolder = instances_path + std::to_string(i * kNthObstacle) + "/";
    for (size_t j = 0; j < kNoInstances; ++j) {
      std::stringstream ss;
      ss << std::setw(4) << std::setfill('0') << j;
      if (!loadPWHFromFile(subfolder + ss.str() + ".yaml", &(*polys)[i][j]))
        return false;
    }
  }

  return true;
}

template <class StripmapPlanner>
typename StripmapPlanner::Settings createSettings(
    Polygon poly, const DecompositionType& decom) {
  typename StripmapPlanner::Settings settings;
  settings.polygon = poly;
  settings.path_cost_function = std::bind(&computeVelocityRampPathCost,
                                          std::placeholders::_1, kVMax, kAMax);
  settings.sensor_model = std::make_shared<Line>(kSweepDistance, kOverlap);
  settings.sweep_around_obstacles = false;
  settings.offset_polygons = true;
  settings.decomposition_type = decom;

  return settings;
}

template <class StripmapPlanner>
bool runPlanner(StripmapPlanner* planner) {
  // Setup.
  timing::Timer timer_setup_total("timer_setup_total");
  planner->setup();
  if (!planner->isInitialized()) return false;
  timer_setup_total.Stop();
  // Solve.
  timing::Timer timer_solve_total("timer_solve_total");
  std::vector<Point_2> solution;
  if (!planner->solve(kStart, kGoal, &solution)) return false;
  timer_solve_total.Stop();
  // TODO(rikba): Save results.
  timing::Timing::Print(std::cout);
  std::cout << "Path cost: "
            << computeVelocityRampPathCost(solution, kVMax, kAMax);
  return true;
}

TEST(BenchmarkTest, Benchmark) {
  std::vector<std::vector<Polygon>> polys;
  // Load polygons.
  ROS_INFO_STREAM("Loading " << kObstacleBins * kNoInstances
                             << " test instances.");
  EXPECT_TRUE(loadAllInstances(&polys));

  // Run planners.
  for (size_t i = 0; i < polys.size(); ++i) {
    for (size_t j = 0; j < polys[i].size(); ++j) {
      // Create settings.
      ROS_INFO_STREAM("Polygon number: " << j);
      if (i > polys[i][j].getPolygon().number_of_holes())
        ROS_WARN_STREAM("Less holes: "
                        << polys[i][j].getPolygon().number_of_holes() << " vs. "
                        << i);
      PolygonStripmapPlanner::Settings our_bcd_settings =
          createSettings<PolygonStripmapPlanner>(
              polys[i][j], DecompositionType::kBoustrophedeon);
      EXPECT_TRUE(runPlanner<PolygonStripmapPlanner>);
      // Create planners.
      PolygonStripmapPlanner our_bcd(our_bcd_settings);
      // Run planners.
      EXPECT_TRUE(runPlanner<PolygonStripmapPlanner>(&our_bcd));
    }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  return RUN_ALL_TESTS();
}
