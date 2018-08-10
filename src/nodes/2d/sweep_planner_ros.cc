#include "mav_coverage_planning/sweep_planner_ros.h"

#include "mav_coverage_planning/common.h"
#include "mav_coverage_planning/msg_from_xml_rpc.h"
#include "mav_coverage_planning/ros_interface.h"

namespace mav_coverage_planning {

SweepPlannerRos::SweepPlannerRos(const ros::NodeHandle& nh,
                                 const ros::NodeHandle& nh_private)
    : BasePlannerRos(nh, nh_private),
      fov_camera_rad_(kDefaultFOVCameraRad),
      min_view_overlap_(kDefaultMinViewOverlap),
      solver_(kDefaultSolver) {
  // Sweep planner specific ROS interaction.
  output_prefix_ = kOutputPrefix + "sweep_planner_ros]: ";

  // Parameters.
  if (!nh_private_.getParam("fov_camera_rad", fov_camera_rad_)) {
    ROS_WARN_STREAM(
        output_prefix_
        << "No camera field of view specified. Using default value of: "
        << fov_camera_rad_);
  }
  if (!nh_private_.getParam("min_view_overlap", min_view_overlap_)) {
    ROS_WARN_STREAM(output_prefix_
                    << "No minimum overlap specified. Using default value of: "
                    << min_view_overlap_);
  }

  int solver = -1;
  if (nh_private_.getParam("solver", solver) &&
      (solver == GTSPPSolver::kExactWithPreprocessing ||
       solver == GTSPPSolver::kExactOnline ||
       solver == GTSPPSolver::kApproximateMA)) {
    ROS_INFO_STREAM(output_prefix_ << "Setting solver to: " << solver);
    solver_ = static_cast<GTSPPSolver>(solver);
  } else {
    ROS_INFO_STREAM(output_prefix_ << "No valid solver set. Using default:"
                                   << solver_);
  }

// Creating the line sweep planner from the retrieved parameters.
// This operation may take some time.
resetPlanner();
}

bool SweepPlannerRos::solvePlanner(const Eigen::Vector2d& start,
                                   const Eigen::Vector2d& goal) {
  return planner_->solve(start, goal, &solution_);
}

bool SweepPlannerRos::resetPlanner() {
  ROS_INFO_STREAM(output_prefix_ << "Reset planner.");
  planner_.reset(new SweepPlanner(polygon_, cost_function_, altitude_,
                                  fov_camera_rad_, min_view_overlap_, solver_));
  if (planner_->isInitialized()) {
    ROS_INFO_STREAM(output_prefix_
                    << "Finished creating the sweep planner.");
    return true;
  } else {
    ROS_ERROR_STREAM(output_prefix_
                     << "Failed creating sweep planner from user input.");
    return false;
  }
}

}  // namespace mav_coverage_planning
