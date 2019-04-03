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
const std::string kResultsFile = "/tmp/coverage_results.txt";
const size_t kMaxNoObstacles = 5;
const size_t kNthObstacle = 5;
const size_t kObstacleBins = kMaxNoObstacles / kNthObstacle + 1;
const size_t kNoInstances = 10;
const double kSweepDistance = 3.0;
const double kOverlap = 0.0;
const double kVMax = 3.0;
const double kAMax = 1.0;
const Point_2 kStart(0.0, 0.0);
const Point_2 kGoal = kStart;
const double kMapScale = 1.0;

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
    std::string subfolder =
        instances_path + std::to_string(i * kNthObstacle) + "/";
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

struct Result {
  std::string planner;
  size_t num_holes;
  size_t num_hole_vertices;
  double cost;
  double sweep_distance = kSweepDistance;
  double v_max = kVMax;
  double a_max = kAMax;
  size_t num_nodes;
  size_t num_edges;
  size_t num_cells;
  std::map<std::string, double> times;
};

bool resultsToCsv(const std::string& path, const std::vector<Result>& results) {
  ROS_INFO_STREAM("Saving results to: " << path);
  std::ofstream file;
  file.open(path);
  if (!file.is_open()) return false;
  file << "planner"
       << ",";
  file << "num_holes"
       << ",";
  file << "num_hole_vertices"
       << ",";
  file << "cost"
       << ",";
  file << "sweep_distance"
       << ",";
  file << "v_max"
       << ",";
  file << "a_max"
       << ",";
  file << "num_nodes"
       << ",";
  file << "num_edges"
       << ",";
  file << "num_cells"
       << ",";
  const Result& first_result = results.front();
  for (std::map<std::string, double>::const_iterator it =
           first_result.times.begin();
       it != first_result.times.end(); ++it) {
    file << it->first;
    if (it != std::prev(first_result.times.end())) file << ",";
  }
  file << "\n";
  for (const Result& result : results) {
    file << result.planner << ",";
    file << result.num_holes << ",";
    file << result.num_hole_vertices << ",";
    file << result.cost << ",";
    file << result.sweep_distance << ",";
    file << result.v_max << ",";
    file << result.a_max << ",";
    file << result.num_nodes << ",";
    file << result.num_edges << ",";
    file << result.num_cells << ",";

    for (std::map<std::string, double>::const_iterator it =
             result.times.begin();
         it != result.times.end(); ++it) {
      file << it->second;
      if (it != std::prev(result.times.end())) file << ",";
    }

    file << "\n";
  }

  file.close();
  return true;
}

void saveTimes(Result* result) {
  timing::Timing::map_t timers = timing::Timing::GetTimers();
  for (timing::Timing::map_t::const_iterator it = timers.begin();
       it != timers.end(); ++it) {
    result->times[it->first] = timing::Timing::GetTotalSeconds(it->second);
  }
}

template <class StripmapPlanner>
bool runPlanner(StripmapPlanner* planner, Result* result) {
  CHECK_NOTNULL(planner);
  CHECK_NOTNULL(result);
  ROS_INFO_STREAM("Planning with: " << result->planner);

  // Setup.
  timing::Timing::Reset();
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
  result->cost = computeVelocityRampPathCost(solution, kVMax, kAMax);
  saveTimes(result);
  result->num_cells = planner->getDecompositionSize();
  result->num_nodes = planner->getNumberOfNodes();
  result->num_edges = planner->getNumberOfEdges();

  // Get times.
  timing::Timing::Print(std::cout);
  ROS_INFO_STREAM("Path cost: " << result->cost);
  return true;
}

TEST(BenchmarkTest, Benchmark) {
  std::vector<std::vector<Polygon>> polys;
  std::vector<Result> results;

  // Load polygons.
  ROS_INFO_STREAM("Loading " << kObstacleBins * kNoInstances
                             << " test instances.");
  EXPECT_TRUE(loadAllInstances(&polys));

  // Run planners.
  for (size_t i = 0; i < polys.size(); ++i) {
    ROS_INFO_STREAM("Number of holes: " << i * kNthObstacle);
    for (size_t j = 0; j < polys[i].size(); ++j) {
      ROS_INFO_STREAM("Polygon number: " << j);

      // Number of hole vertices.
      size_t num_hole_vertices = computeNoHoleVertices(polys[i][j]);
      size_t num_holes = polys[i][j].getPolygon().number_of_holes();
      EXPECT_EQ(i * kNthObstacle, polys[i][j].getPolygon().number_of_holes());

      // Create results.
      Result our_bcd_result;
      our_bcd_result.num_holes = num_holes;
      our_bcd_result.num_hole_vertices = num_hole_vertices;
      our_bcd_result.planner = "our_bcd";

      Result our_tcd_result;
      our_tcd_result.num_holes = num_holes;
      our_tcd_result.num_hole_vertices = num_hole_vertices;
      our_tcd_result.planner = "our_tcd";

      // Create settings.
      PolygonStripmapPlanner::Settings our_bcd_settings =
          createSettings<PolygonStripmapPlanner>(
              polys[i][j], DecompositionType::kBoustrophedeon);
      PolygonStripmapPlanner::Settings our_tcd_settings =
          createSettings<PolygonStripmapPlanner>(
              polys[i][j], DecompositionType::kTrapezoidal);
      // Create planners.
      PolygonStripmapPlanner our_bcd(our_bcd_settings);
      PolygonStripmapPlanner our_tcd(our_tcd_settings);
      // Run planners.
      EXPECT_TRUE(
          runPlanner<PolygonStripmapPlanner>(&our_bcd, &our_bcd_result));
      EXPECT_TRUE(
          runPlanner<PolygonStripmapPlanner>(&our_tcd, &our_tcd_result));

      // Save results.
      results.push_back(our_bcd_result);
      results.push_back(our_tcd_result);
    }
  }

  EXPECT_TRUE(resultsToCsv(kResultsFile, results));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  return RUN_ALL_TESTS();
}
