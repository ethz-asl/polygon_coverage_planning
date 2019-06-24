#include <ros/assert.h>
#include <ros/console.h>
#include <cmath>

#include <polygon_coverage_geometry/cgal_comm.h>

#include "polygon_coverage_planners/planners/polygon_stripmap_planner.h"
#include "polygon_coverage_planners/timing.h"

namespace polygon_coverage_planning {

PolygonStripmapPlanner::PolygonStripmapPlanner(
    const sweep_plan_graph::SweepPlanGraph::Settings& settings)
    : is_initialized_(false), settings_(settings) {}

bool PolygonStripmapPlanner::setup() {
  is_initialized_ = true;

  // Create sweep plan graph.
  timing::Timer timer_sweep_graph("sweep_graph");
  if (is_initialized_) {
    ROS_INFO("Start creating sweep plan graph.");
    sweep_plan_graph_ = sweep_plan_graph::SweepPlanGraph(settings_);
    if (!sweep_plan_graph_.isInitialized()) {
      ROS_ERROR("Cannot create sweep plan graph.");
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
  ROS_ASSERT(solution);
  solution->clear();

  if (!is_initialized_) {
    ROS_ERROR(
        "Could not create sweep planner for user input. Failed to compute "
        "solution.");
    return false;
  }

  // Make sure start and end are inside the settings_.polygon.
  const Point_2 start_new = pointInPolygon(settings_.polygon, start)
                                ? start
                                : projectPointOnHull(settings_.polygon, start);
  const Point_2 goal_new = pointInPolygon(settings_.polygon, goal)
                               ? goal
                               : projectPointOnHull(settings_.polygon, goal);

  if (!runSolver(start_new, goal_new, solution)) {
    ROS_ERROR("Failed solving graph.");
    return false;
  }

  // Make sure original start and end are part of the plan.
  if (!pointInPolygon(settings_.polygon, start)) {
    solution->insert(solution->begin(), start);
  }
  if (!pointInPolygon(settings_.polygon, goal)) {
    solution->insert(solution->end(), goal);
  }

  timer_solve.Stop();

  return true;
}

bool PolygonStripmapPlanner::runSolver(const Point_2& start,
                                       const Point_2& goal,
                                       std::vector<Point_2>* solution) const {
  ROS_ASSERT(solution);

  ROS_INFO("Start solving GTSP using GK MA.");
  return sweep_plan_graph_.solve(start, goal, solution);
}

}  // namespace polygon_coverage_planning
