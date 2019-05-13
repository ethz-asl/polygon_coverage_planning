#include "mav_2d_coverage_planning/planners/polygon_stripmap_planner.h"
#include <CGAL/Boolean_set_operations_2.h>
#include <glog/logging.h>
#include <cmath>

#include <mav_coverage_planning_comm/timing.h>

namespace mav_coverage_planning {

PolygonStripmapPlanner::PolygonStripmapPlanner(const Settings& settings)
    : is_initialized_(false), settings_(settings) {}

bool PolygonStripmapPlanner::setup() {
  is_initialized_ = true;

  // Create sweep plan graph.
  timing::Timer timer_sweep_graph("sweep_graph");
  CHECK_NOTNULL(settings_.sensor_model);
  if (is_initialized_) {
    LOG(INFO) << "Start creating sweep plan graph.";
    sweep_plan_graph_ = sweep_plan_graph::SweepPlanGraph(
        settings_.polygon, settings_.path_cost_function, decomposition_,
        settings_.sensor_model->getSweepDistance(),
        settings_.sweep_single_direction);
    if (!sweep_plan_graph_.isInitialized()) {
      LOG(ERROR) << "Cannot create sweep plan graph.";
      is_initialized_ = false;
    }
  }
  timer_sweep_graph.Stop();

  // Solver specific setup.
  timing::Timer timer_setup_solver("setup_solver");
  is_initialized_ = setupSolver();
  timer_setup_solver.Stop();

  return is_initialized_;
}

bool PolygonStripmapPlanner::solve(const Point_2& start, const Point_2& goal,
                                   std::vector<Point_2>* solution) const {
  timing::Timer timer_solve("solve");
  CHECK_NOTNULL(solution);
  solution->clear();

  if (!is_initialized_) {
    LOG(ERROR) << "Could not create sweep planner for user input. Failed to "
                  "compute solution.";
    return false;
  }

  // Make sure start and end are inside the settings_.polygon.
  const Point_2 start_new = settings_.polygon.pointInPolygon(start)
                                ? start
                                : settings_.polygon.projectPointOnHull(start);
  const Point_2 goal_new = settings_.polygon.pointInPolygon(goal)
                               ? goal
                               : settings_.polygon.projectPointOnHull(goal);

  if (!runSolver(start_new, goal_new, solution)) {
    LOG(ERROR) << "Failed solving graph.";
    return false;
  }

  // TODO(rikba): Make this part of the optimization.
  if (settings_.sweep_around_obstacles) {
    if (!sweepAroundObstacles(solution)) {
      LOG(ERROR) << "Failed sweeping around obstacles.";
      return false;
    }
  }

  // Make sure original start and end are part of the plan.
  if (!settings_.polygon.pointInPolygon(start)) {
    solution->insert(solution->begin(), start);
  }
  if (!settings_.polygon.pointInPolygon(goal)) {
    solution->insert(solution->end(), goal);
  }

  timer_solve.Stop();

  return true;
}

bool PolygonStripmapPlanner::sweepAroundObstacles(
    std::vector<Point_2>* solution) const {
  Point_2 goal = solution->back();
  solution->pop_back();
  std::vector<Point_2> waypoints;
  visibility_graph::VisibilityGraph visibility_graph(settings_.polygon);
  std::vector<std::vector<Point_2>> holes = settings_.polygon.getHoleVertices();

  for (size_t i = 0u; i < holes.size(); ++i) {
    std::vector<Point_2> hole = holes[i];
    waypoints.clear();
    visibility_graph.solve(solution->back(), hole.front(), &waypoints);
    solution->insert(solution->end(), waypoints.begin() + 1, waypoints.end());
  }

  std::vector<Point_2> hull = settings_.polygon.getHullVertices();
  waypoints.clear();
  visibility_graph.solve(solution->back(), hull.front(), &waypoints);
  solution->insert(solution->end(), waypoints.begin() + 1, waypoints.end());

  waypoints.clear();
  visibility_graph.solve(solution->back(), goal, &waypoints);
  solution->insert(solution->end(), waypoints.begin() + 1, waypoints.end() - 1);
  solution->push_back(goal);
  return true;
}

bool PolygonStripmapPlanner::runSolver(const Point_2& start,
                                       const Point_2& goal,
                                       std::vector<Point_2>* solution) const {
  CHECK_NOTNULL(solution);

  LOG(INFO) << "Start solving GTSP using GK MA.";
  return sweep_plan_graph_.solve(start, goal, solution);
}

}  // namespace mav_coverage_planning
