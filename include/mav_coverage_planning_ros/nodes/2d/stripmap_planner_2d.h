#ifndef MAV_COVERAGE_PLANNING_ROS_STRIPMAP_PLANNER_2D_H_
#define MAV_COVERAGE_PLANNING_ROS_STRIPMAP_PLANNER_2D_H_

#include <memory>

#include <ros/ros.h>

#include <mav_2d_coverage_planning/sensor_models/frustum.h>
#include <mav_2d_coverage_planning/sensor_models/line.h>

#include <mav_coverage_planning_ros/conversions/ros_interface.h>
#include "mav_coverage_planning_ros/nodes/2d/base_planner_2d.h"

namespace mav_coverage_planning {

// A ros wrapper for the line sweep planner
template <class StripmapPlanner>
class StripmapPlanner2D : public BasePlanner2D {
 public:
  // Constructor
  StripmapPlanner2D(const ros::NodeHandle& nh,
                    const ros::NodeHandle& nh_private)
      : BasePlanner2D(nh, nh_private),
        decomposition_type_(DecompositionType::kBoustrophedeon),
        sweep_around_obstacles_(false),
        offset_polygons_(true),
        sweep_single_direction_(false) {
    // Parameters.
    double lateral_overlap = 0.0;
    if (!nh_private_.getParam("lateral_overlap", lateral_overlap)) {
      ROS_WARN_STREAM("No lateral overlap specified. Using default value of: "
                      << lateral_overlap);
    }

    if (!nh_private_.getParam("sweep_around_obstacles",
                              sweep_around_obstacles_)) {
      ROS_WARN_STREAM(
          "Not defined if robot should sweep around obstacles. Using default "
          "value of: "
          << sweep_around_obstacles_);
    }

    if (!nh_private_.getParam("offset_polygons", offset_polygons_)) {
      ROS_WARN_STREAM(
          "Not defined if decomposition polygons should be offsetted. Using "
          "default value of: "
          << offset_polygons_);
    }

    int decomposition_type_int = 0;
    if (!nh_private_.getParam("decomposition_type", decomposition_type_int)) {
      ROS_WARN_STREAM(
          "No decomposition type specified. Using default value of: "
          << decomposition_type_int);
    }
    decomposition_type_ =
        static_cast<DecompositionType>(decomposition_type_int);

    // Get sensor model.
    int sensor_model_type_int = 0;
    if (!nh_private_.getParam("sensor_model_type", sensor_model_type_int)) {
      ROS_WARN_STREAM("No sensor model type specified. Using default value of: "
                      << sensor_model_type_int);
    }
    SensorModelType sensor_model_type =
        static_cast<SensorModelType>(sensor_model_type_int);
    switch (sensor_model_type) {
      case SensorModelType::kLine: {
        double lateral_footprint = 1.0;
        if (!nh_private_.getParam("lateral_footprint", lateral_footprint)) {
          ROS_WARN_STREAM(
              "No lateral footprint specified. Using default value of: "
              << lateral_footprint);
        }
        sensor_model_ =
            std::make_shared<Line>(lateral_footprint, lateral_overlap);
        ROS_INFO("Sensor model: line");
        ROS_INFO_STREAM("Lateral footprint: " << lateral_footprint);
        break;
      }
      case SensorModelType::kFrustum:
      default: {
        double lateral_fov = 0.5 * M_PI;
        if (!nh_private_.getParam("lateral_fov", lateral_fov)) {
          ROS_WARN_STREAM("No lateral fov specified. Using default value of: "
                          << lateral_fov);
        }
        sensor_model_ = std::make_shared<Frustum>(settings_.altitude,
                                                  lateral_fov, lateral_overlap);
        ROS_INFO("Sensor model: frustum");
        ROS_INFO_STREAM("Lateral FOV: " << lateral_fov);
        ROS_INFO_STREAM("Altitude: " << settings_.altitude);
        break;
      }
    }
    ROS_INFO_STREAM("Lateral overlap: " << lateral_overlap);
    ROS_INFO_STREAM("Sweep distance: " << sensor_model_->getSweepDistance());


    if (!nh_private_.getParam("sweep_single_direction", sweep_single_direction_)) {
      ROS_WARN_STREAM("Default sweeping in single direction: " << sweep_single_direction_);
    }
    ROS_INFO_STREAM("Sweep single direction: " << sweep_single_direction_);

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
    settings.polygon = settings_.polygon_offset;
    settings.path_cost_function = settings_.sweep_cost_function;
    settings.sensor_model = sensor_model_;
    settings.sweep_around_obstacles = sweep_around_obstacles_;
    settings.offset_polygons = offset_polygons_;
    settings.decomposition_type = decomposition_type_;
    settings.sweep_single_direction = sweep_single_direction_;

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

  inline visualization_msgs::MarkerArray createDecompositionMarkers()
      const override {
    visualization_msgs::MarkerArray markers;
    if (!planner_) {
      return markers;
    }

    std::vector<Polygon> decomposition = planner_->getDecomposition();
    for (size_t i = 0; i < decomposition.size(); ++i) {
      visualization_msgs::MarkerArray decomposition_markers;
      std::string name = "decomposition_polygon_" + std::to_string(i);
      const double kPolygonLineSize = 0.4;
      createPolygonMarkers(
          decomposition[i], settings_.altitude, settings_.global_frame_id, name,
          mav_visualization::Color::Gray(), mav_visualization::Color::Gray(),
          kPolygonLineSize, kPolygonLineSize, &decomposition_markers);
      markers.markers.insert(markers.markers.end(),
                             decomposition_markers.markers.begin(),
                             decomposition_markers.markers.end());
    }

    return markers;
  }

  // The library object that actually does planning.
  std::unique_ptr<StripmapPlanner> planner_;

  // System Parameters
  std::shared_ptr<SensorModelBase> sensor_model_;
  DecompositionType decomposition_type_;
  bool sweep_around_obstacles_;
  bool offset_polygons_;
  bool sweep_single_direction_;
};
}  // namespace mav_coverage_planning

#endif  // MAV_COVERAGE_PLANNING_ROS_STRIPMAP_PLANNER_2D_H_
