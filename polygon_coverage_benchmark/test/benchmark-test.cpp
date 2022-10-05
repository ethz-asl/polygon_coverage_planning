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

#include <iomanip>
#include <sstream>

#include <CGAL/Boolean_set_operations_2.h>
#include <ros/assert.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <polygon_coverage_msgs/PolygonWithHoles.h>

#include <polygon_coverage_geometry/cgal_definitions.h>
#include <polygon_coverage_planners/cost_functions/path_cost_functions.h>
#include <polygon_coverage_planners/graphs/sweep_plan_graph.h>
#include <polygon_coverage_planners/planners/polygon_stripmap_planner.h>
#include <polygon_coverage_planners/planners/polygon_stripmap_planner_exact.h>
#include <polygon_coverage_planners/sensor_models/line.h>
#include <polygon_coverage_planners/timing.h>

const std::string kPackageName = "polygon_coverage_ros";
const std::string kResultsFile = "/tmp/coverage_results.txt";
const size_t kMaxNoObstacles = 15;
const size_t kNthObstacle = 1;
const size_t kObstacleBins = kMaxNoObstacles / kNthObstacle + 1;
const size_t kNoInstances = 20;
const double kSweepDistance = 3.0;
const double kOverlap = 0.0;
const double kVMax = 3.0;
const double kAMax = 1.0;
const Point_2 kStart(0.0, 0.0);
const Point_2 kGoal = kStart;
const double kMapScale = 1.0;

using namespace polygon_coverage_planning;

bool loadPolygonFromNode(const YAML::Node& node, Polygon_2* poly) {
  ROS_ASSERT(poly);
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

bool loadPWHFromFile(const std::string& file, PolygonWithHoles* polygon) {
  ROS_ASSERT(polygon);

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

  ROS_ASSERT(polygon);
  *polygon = PolygonWithHoles(pwh);

  return true;
}

size_t computeNoHoleVertices(const PolygonWithHoles& poly) {
  size_t no_hole_vertices = 0;
  for (PolygonWithHoles::Hole_const_iterator hit =
           poly.holes_begin();
       hit != poly.holes_end(); ++hit) {
    no_hole_vertices += hit->size();
  }
  return no_hole_vertices;
}

bool loadAllInstances(std::vector<PolygonWithHoles>* polys,
                      std::vector<std::string>* names) {
  ROS_ASSERT(polys);
  ROS_ASSERT(names);
  polys->reserve(kObstacleBins * kNoInstances);
  names->reserve(kObstacleBins * kNoInstances);

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
      polys->push_back(PolygonWithHoles());
      if (!loadPWHFromFile(subfolder + ss.str() + ".yaml", &polys->back()))
        return false;
      names->push_back(std::to_string(i * kNthObstacle) + "/" + ss.str());
    }
  }

  return true;
}

sweep_plan_graph::SweepPlanGraph::Settings createSettings(
    PolygonWithHoles poly, const DecompositionType& decom, bool sweep_single_direction) {
  sweep_plan_graph::SweepPlanGraph::Settings settings;
  settings.polygon = poly;
  settings.cost_function = std::bind(&computeVelocityRampPathCost,
                                          std::placeholders::_1, kVMax, kAMax);
  settings.sensor_model = std::make_shared<Line>(kSweepDistance, kOverlap);
  settings.offset_polygons = true;
  settings.decomposition_type = decom;
  settings.sweep_single_direction = sweep_single_direction;

  return settings;
}

struct Result {
  std::string planner;
  std::string instance;
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

bool initCsv(const std::string& path, const Result& result) {
  ROS_INFO_STREAM("Init results file: " << path);
  std::ofstream file;
  file.open(path);
  if (!file.is_open()) return false;
  file << "planner"
       << ",";
  file << "instance"
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
  for (std::map<std::string, double>::const_iterator it = result.times.begin();
       it != result.times.end(); ++it) {
    file << it->first;
    if (it != std::prev(result.times.end())) file << ",";
  }
  file << "\n";
  file.close();
  return true;
}

bool resultToCsv(const std::string& path, const Result& result) {
  std::ofstream file;
  file.open(path, std::fstream::app);
  if (!file.is_open()) return false;

  file << result.planner << ",";
  file << result.instance << ",";
  file << result.num_holes << ",";
  file << result.num_hole_vertices << ",";
  file << result.cost << ",";
  file << result.sweep_distance << ",";
  file << result.v_max << ",";
  file << result.a_max << ",";
  file << result.num_nodes << ",";
  file << result.num_edges << ",";
  file << result.num_cells << ",";

  for (std::map<std::string, double>::const_iterator it = result.times.begin();
       it != result.times.end(); ++it) {
    file << it->second;
    if (it != std::prev(result.times.end())) file << ",";
  }

  file << "\n";

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
  ROS_ASSERT(planner);
  ROS_ASSERT(result);
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
  // Save results.
  result->cost = computeVelocityRampPathCost(solution, kVMax, kAMax);
  saveTimes(result);
  //TODO(stlucas): may change protection of these stats
  //result->num_cells = planner->settings_.getDecompositionSize();
  //result->num_nodes = planner->sweep_plan_graph_.size();
  //result->num_edges = planner->sweep_plan_graph_.getNumberOfEdges();

  // Get times.
  timing::Timing::Print(std::cout);
  ROS_INFO_STREAM("Path cost: " << result->cost);
  return true;
}

void runBenchmark() {
  std::vector<PolygonWithHoles> polys;
  std::vector<std::string> names;

  // Load polygons.
  ROS_INFO_STREAM("Loading " << kObstacleBins * kNoInstances
                             << " test instances.");
  if(!loadAllInstances(&polys, &names))
  {
        ROS_ERROR("Failed to load all instances");
        return;
  }

  // Run planners.
  for (size_t i = 0; i < polys.size(); ++i) {
    ROS_INFO_STREAM("Polygon number: " << i);
    ROS_INFO_STREAM("Polygon name: " << names[i]);

    // Number of hole vertices.
    size_t num_hole_vertices = computeNoHoleVertices(polys[i]);
    size_t num_holes = polys[i].number_of_holes();
    ROS_INFO_STREAM("Number of holes: " << num_holes);

    // Create results.
    Result our_bcd_result;
    our_bcd_result.num_holes = num_holes;
    our_bcd_result.num_hole_vertices = num_hole_vertices;
    our_bcd_result.planner = "our_bcd";
    our_bcd_result.instance = names[i];

    Result our_tcd_result = our_bcd_result;
    our_tcd_result.planner = "our_tcd";

    Result one_dir_gkma_result = our_bcd_result;
    one_dir_gkma_result.planner = "one_dir_gkma";

    Result gtsp_exact_result = our_bcd_result;
    gtsp_exact_result.planner = "gtsp_exact";

    Result one_dir_exact_result = our_bcd_result;
    one_dir_exact_result.planner = "one_dir_exact";

    // Create settings.
    sweep_plan_graph::SweepPlanGraph::Settings our_bcd_settings =
        createSettings(polys[i],
                                               DecompositionType::kBCD, false);
    sweep_plan_graph::SweepPlanGraph::Settings our_tcd_settings =
        createSettings(
            polys[i], DecompositionType::kTCD, false);
    sweep_plan_graph::SweepPlanGraph::Settings one_dir_gkma_settings =
        createSettings(polys[i],
                                               DecompositionType::kBCD, true);
    sweep_plan_graph::SweepPlanGraph::Settings gtsp_exact_settings =
        createSettings(polys[i],
                                               DecompositionType::kBCD, false);
    sweep_plan_graph::SweepPlanGraph::Settings one_dir_exact_settings =
        createSettings(polys[i],
                                               DecompositionType::kBCD, true);

    // Create planners.
    PolygonStripmapPlanner our_bcd(our_bcd_settings);
    PolygonStripmapPlanner our_tcd(our_tcd_settings);
    PolygonStripmapPlanner one_dir_gkma(one_dir_gkma_settings);
    PolygonStripmapPlannerExact gtsp_exact(gtsp_exact_settings);
    PolygonStripmapPlannerExact one_dir_exact(one_dir_exact_settings);

    // Run planners.
    ROS_ASSERT(runPlanner<PolygonStripmapPlanner>(&our_bcd, &our_bcd_result));
    ROS_ASSERT(runPlanner<PolygonStripmapPlanner>(&our_tcd, &our_tcd_result));
    ROS_ASSERT(runPlanner<PolygonStripmapPlanner>(&one_dir_gkma,
                                                   &one_dir_gkma_result));

    bool success_gtsp_exact = false;
    bool success_one_dir_exact = false;

    if (num_holes < 3) {
      success_gtsp_exact = runPlanner<PolygonStripmapPlannerExact>(
         &gtsp_exact, &gtsp_exact_result);
      success_one_dir_exact = runPlanner<PolygonStripmapPlannerExact>(
         &one_dir_exact, &one_dir_exact_result);
    }

    // Save results.
    if (i == 0) ROS_ASSERT(initCsv(kResultsFile, our_bcd_result));

    ROS_ASSERT(resultToCsv(kResultsFile, our_bcd_result));
    ROS_ASSERT(resultToCsv(kResultsFile, our_tcd_result));
    ROS_ASSERT(resultToCsv(kResultsFile, one_dir_gkma_result));

    if (success_gtsp_exact)
      ROS_ASSERT(resultToCsv(kResultsFile, gtsp_exact_result));
    if (success_one_dir_exact)
      ROS_ASSERT(resultToCsv(kResultsFile, one_dir_exact_result));
  }

}

int main(int argc, char** argv) {
  runBenchmark();
  return 0;
}
