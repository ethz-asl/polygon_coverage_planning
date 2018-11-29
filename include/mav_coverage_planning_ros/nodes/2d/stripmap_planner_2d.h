#ifndef MAV_COVERAGE_PLANNING_ROS_STRIPMAP_PLANNER_2D_H_
#define MAV_COVERAGE_PLANNING_ROS_STRIPMAP_PLANNER_2D_H_

#include <memory>

#include <ros/ros.h>

#include "mav_coverage_planning_ros/nodes/2d/base_planner_2d.h"

namespace mav_coverage_planning {

// The default width of robot
constexpr double kDefaultRobotSize = 1.0;
// The default minimum view overlap between two sweeps [0 .. 1)
constexpr double kDefaultMinViewOverlap = 0.0;
constexpr double kDefaultWallDist = 0.0;
constexpr bool kDefaultSweepAroundObstacles = false;
// A ros wrapper for the line sweep planner
template <class StripmapPlanner>
class StripmapPlanner2D : public BasePlanner2D {
 public:
  // Constructor
  StripmapPlanner2D(const ros::NodeHandle& nh,
                    const ros::NodeHandle& nh_private)
      : BasePlanner2D(nh, nh_private),
        robot_size_(kDefaultRobotSize),
        wall_dist_(kDefaultWallDist),
        min_view_overlap_(kDefaultMinViewOverlap),
        sweep_around_obstacles_(kDefaultSweepAroundObstacles)
        {

    // Parameters.
    if (!nh_private_.getParam("robot_size", robot_size_)) {
      ROS_WARN_STREAM(
          "No robot size specified. Using default value of: " << robot_size_);
    }
    if (!nh_private_.getParam("wall_dist", wall_dist_)) {
      ROS_WARN_STREAM(
          "No robot size specified. Using default value of: " << wall_dist_);
    }
    if (!nh_private_.getParam("min_view_overlap", min_view_overlap_)) {
      ROS_WARN_STREAM("No minimum overlap specified. Using default value of: "
                      << min_view_overlap_);
    }
    
    if (!nh_private_.getParam("sweep_around_obstacles", sweep_around_obstacles_)) {
      ROS_WARN_STREAM("Not defined if robot should sweep around obstacles. Using default value of: "
                      << sweep_around_obstacles_);
    }

    // Creating the line sweep planner from the retrieved parameters.
    // This operation may take some time.
    resetPlanner();
  }

 private:
  // Call to the sweep planner library.
  inline bool solvePlanner(const Point_2& start, const Point_2& goal) override {
    return planner_->solve(start, goal, &solution_);
  }

  // Reset the sweep planner when a new polygon is set.
  inline bool resetPlanner() override {
    ROS_INFO_STREAM("Reset planner.");
    typename StripmapPlanner::Settings settings;
    settings.polygon = settings_.polygon;
    settings.path_cost_function = settings_.sweep_cost_function;
    settings.robot_size = robot_size_;
    settings.wall_dist = wall_dist_;
    settings.min_view_overlap = min_view_overlap_;
    settings.sweep_around_obstacles = sweep_around_obstacles_;

    planner_.reset(new StripmapPlanner(settings));
    planner_->setup();
    if (planner_->isInitialized()) {
      ROS_INFO_STREAM("Finished creating the sweep planner.");
      return true;
    } else {
      ROS_ERROR_STREAM("Failed creating sweep planner from user input.");
      return false;
    }
  }

  // The library object that actually does planning.
  std::unique_ptr<StripmapPlanner> planner_;

  // System Parameters
  double robot_size_;
  double wall_dist_;
  double min_view_overlap_;
  bool sweep_around_obstacles_;
};
}  // namespace mav_coverage_planning

#endif  // MAV_COVERAGE_PLANNING_ROS_STRIPMAP_PLANNER_2D_H_
