#ifndef MAV_COVERAGE_PLANNING_ROS_STRIPMAP_PLANNER_2D_H_
#define MAV_COVERAGE_PLANNING_ROS_STRIPMAP_PLANNER_2D_H_

#include <memory>

#include <ros/ros.h>

#include "mav_coverage_planning_ros/nodes/2d/base_planner_2d.h"

namespace mav_coverage_planning {

// The default FOV of the camera
constexpr double kDefaultFOVCameraRad = M_PI / 2.0;
// The default minimum view overlap between two sweeps [0 .. 1)
constexpr double kDefaultMinViewOverlap = 0.2;

// A ros wrapper for the line sweep planner
template <class StripmapPlanner>
class StripmapPlanner2D : public BasePlanner2D {
 public:
  // Constructor
  StripmapPlanner2D(const ros::NodeHandle& nh,
                    const ros::NodeHandle& nh_private)
      : BasePlanner2D(nh, nh_private),
        lateral_fov_(kDefaultFOVCameraRad),
        longitudinal_fov_(kDefaultFOVCameraRad),
        min_view_overlap_(kDefaultMinViewOverlap) {
    // Parameters.
    if (!nh_private_.getParam("lateral_fov", lateral_fov_)) {
      ROS_WARN_STREAM(
          "No lateral camera field of view specified. Using default value of: "
          << lateral_fov_);
    }
    if (!nh_private_.getParam("longitudinal_fov", longitudinal_fov_)) {
      ROS_WARN_STREAM(
          "No longitudinal camera field of view specified. Using default value "
          "of: "
          << longitudinal_fov_);
    }
    if (!nh_private_.getParam("min_view_overlap", min_view_overlap_)) {
      ROS_WARN_STREAM("No minimum overlap specified. Using default value of: "
                      << min_view_overlap_);
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
    settings.segment_cost_function = settings_.visibility_graph_cost_function;
    settings.altitude = settings_.altitude;
    settings.lateral_fov = lateral_fov_;
    settings.longitudinal_fov = longitudinal_fov_;
    settings.min_view_overlap = min_view_overlap_;

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
  double lateral_fov_;
  double longitudinal_fov_;
  double min_view_overlap_;
};
}  // namespace mav_coverage_planning

#endif  // MAV_COVERAGE_PLANNING_ROS_STRIPMAP_PLANNER_2D_H_
