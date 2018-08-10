#ifndef MAV_COVERAGE_PLANNING_SWEEP_PLANNING_ROS_H_
#define MAV_COVERAGE_PLANNING_SWEEP_PLANNING_ROS_H_

#include <memory>

#include <Eigen/Core>

#include <ros/ros.h>

#include "mav_coverage_planning/sweep_planner.h"
#include "mav_coverage_planning/base_planner_ros.h"

namespace mav_coverage_planning {

// The default FOV of the camera
constexpr double kDefaultFOVCameraRad = M_PI / 2.0;
// The default minimum view overlap between two sweeps [0 .. 1)
constexpr double kDefaultMinViewOverlap = 0.2;
// The default solver.
const GTSPPSolver kDefaultSolver = GTSPPSolver::kApproximateMA;

// A ros wrapper for the line sweep planner
class SweepPlannerRos : BasePlannerRos {
 public:
  // Constructor
  SweepPlannerRos(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

 private:

  // Call to the sweep planner library.
  virtual bool solvePlanner(const Eigen::Vector2d& start,
                            const Eigen::Vector2d& goal);

  // Reset the sweep planner when a new polygon is set.
  virtual bool resetPlanner();

  // The library object that actually does planning.
  std::unique_ptr<SweepPlanner> planner_;

  // System Parameters
  double fov_camera_rad_;
  double min_view_overlap_;
  GTSPPSolver solver_;
};
}  // namespace mav_coverage_planning

#endif  // MAV_COVERAGE_PLANNING_SWEEP_PLANNING_ROS_H_
