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

#ifndef POLYGON_COVERAGE_ROS_COVERAGE_PLANNER_H_
#define POLYGON_COVERAGE_ROS_COVERAGE_PLANNER_H_

#include <memory>
#include <optional>

#include <ros/ros.h>

#include <polygon_coverage_geometry/decomposition.h>
#include <polygon_coverage_planners/graphs/sweep_plan_graph.h>
#include <polygon_coverage_planners/sensor_models/frustum.h>
#include <polygon_coverage_planners/sensor_models/line.h>
#include <polygon_coverage_planners/sensor_models/sensor_model_base.h>

#include "polygon_coverage_ros/polygon_planner_base.h"
#include "polygon_coverage_ros/ros_interface.h"

namespace polygon_coverage_planning {

// A ros wrapper for the line sweep planner
template <class Planner>
class CoveragePlanner : public PolygonPlannerBase {
 public:
  // Constructor
  CoveragePlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
      : PolygonPlannerBase(nh, nh_private),
        decomposition_type_(DecompositionType::kBCD),
        sensor_model_type_(SensorModelType::kLine),
        offset_polygons_(true),
        sweep_single_direction_(false) {
    // Parameters.
    if (!nh_private_.getParam("offset_polygons", offset_polygons_)) {
      ROS_WARN_STREAM(
          "Not defined if decomposition polygons should be offsetted. Using "
          "default value of: "
          << offset_polygons_);
    }

    // Decomposition type.
    int decomposition_type_int = static_cast<int>(decomposition_type_);
    if (!nh_private_.getParam("decomposition_type", decomposition_type_int)) {
      ROS_WARN_STREAM(
          "No decomposition type specified. Using default value of: "
          << getDecompositionTypeName(decomposition_type_));
    }
    if (!checkDecompositionTypeValid(decomposition_type_int)) {
      ROS_WARN_STREAM(
          "Selected decomposition type is invalid. Using default value of: "
          << getDecompositionTypeName(decomposition_type_));
      decomposition_type_int = static_cast<int>(decomposition_type_);
    }
    decomposition_type_ =
        static_cast<DecompositionType>(decomposition_type_int);

    // Get sensor model.
    int sensor_model_type_int = static_cast<int>(sensor_model_type_);
    if (!nh_private_.getParam("sensor_model_type", sensor_model_type_int)) {
      ROS_WARN_STREAM("No sensor model type specified. Using default value of: "
                      << sensor_model_type_int);
    }
    if (!checkSensorModelTypeValid(sensor_model_type_int)) {
      ROS_WARN_STREAM(
          "Selected sensor model type is invalid. Using default value of: "
          << getSensorModelTypeName(sensor_model_type_));
      sensor_model_type_int = static_cast<int>(sensor_model_type_);
    }
    sensor_model_type_ = static_cast<SensorModelType>(sensor_model_type_int);

    double lateral_footprint_temp = -1.0;
    if (nh_private_.getParam("lateral_footprint", lateral_footprint_temp)) {
      lateral_footprint_ = std::make_optional(lateral_footprint_temp);
    }

    double lateral_overlap_temp = -1.0;
    if (nh_private_.getParam("lateral_overlap", lateral_overlap_temp)) {
      lateral_overlap_ = std::make_optional(lateral_overlap_temp);
    }

    double lateral_fov_temp = -1.0;
    if (nh_private_.getParam("lateral_fov", lateral_fov_temp)) {
      lateral_fov_ = std::make_optional(lateral_fov_temp);
    }

    updateSensorModel();

    if (!nh_private_.getParam("sweep_single_direction",
                              sweep_single_direction_)) {
      ROS_WARN_STREAM(
          "Default sweeping in single direction: " << sweep_single_direction_);
    }
    ROS_INFO_STREAM("Sweep single direction: " << sweep_single_direction_);

    // Creating the line sweep planner from the retrieved parameters.
    // This operation may take some time.
    if (polygon_.has_value()) {
      resetPlanner();
    }
  }

 protected:
  inline visualization_msgs::MarkerArray createDecompositionMarkers()
      const override {
    visualization_msgs::MarkerArray markers;
    if (!planner_) {
      return markers;
    }
    if (!altitude_.has_value()) {
      ROS_WARN("Altitude not set. Cannot create decomposition markers.");
      return markers;
    }

    std::vector<Polygon_2> decomposition = planner_->getDecomposition();
    for (size_t i = 0; i < decomposition.size(); ++i) {
      visualization_msgs::MarkerArray decomposition_markers;
      std::string name = "decomposition_polygon_" + std::to_string(i);
      const double kPolygonLineSize = 0.4;
      createPolygonMarkers(PolygonWithHoles(decomposition[i]),
                           altitude_.value(), global_frame_id_, name,
                           Color::Gray(), Color::Gray(), kPolygonLineSize,
                           kPolygonLineSize, &decomposition_markers);
      markers.markers.insert(markers.markers.end(),
                             decomposition_markers.markers.begin(),
                             decomposition_markers.markers.end());
    }

    return markers;
  }

 private:
  // Call to the sweep planner library.
  inline bool solvePlanner(const Point_2& start, const Point_2& goal) override {
    return planner_->solve(start, goal, &solution_);
  }

  // Reset the sweep planner when a new polygon is set.
  inline bool resetPlanner() override {
    ROS_INFO_STREAM("Reset planner.");
    sweep_plan_graph::SweepPlanGraph::Settings settings;
    if (!polygon_.has_value()) {
      ROS_ERROR("Polygon not set.");
      return false;
    } else {
      settings.polygon = polygon_.value();
    }
    settings.cost_function = path_cost_function_.first;

    updateSensorModel();
    if (sensor_model_ == nullptr) {
      ROS_ERROR("Sensor model not set.");
      return false;
    } else {
      settings.sensor_model = sensor_model_;
    }
    settings.decomposition_type = decomposition_type_;
    settings.wall_distance = wall_distance_;
    settings.offset_polygons = offset_polygons_;
    settings.sweep_single_direction = sweep_single_direction_;

    planner_.reset(new Planner(settings));
    planner_->setup();
    if (planner_->isInitialized()) {
      ROS_INFO("Finished creating the sweep planner.");
      return true;
    } else {
      ROS_ERROR("Failed creating sweep planner from user input.");
      return false;
    }
  }

  inline void updateSensorModel() {
    switch (sensor_model_type_) {
      case SensorModelType::kLine: {
        if (!lateral_footprint_.has_value()) {
          ROS_ERROR("No lateral_footprint specified. Cannot set sensor model.");
          break;
        }
        if (!lateral_overlap_.has_value()) {
          ROS_ERROR("No lateral_overlap specified. Cannot set sensor model.");
          break;
        }
        sensor_model_ = std::make_shared<Line>(lateral_footprint_.value(),
                                               lateral_overlap_.value());
        ROS_INFO_STREAM(
            "Sensor model: " << getSensorModelTypeName(sensor_model_type_));
        ROS_INFO_STREAM("Lateral footprint: " << lateral_footprint_.value());
        break;
      }
      case SensorModelType::kFrustum:
      default: {
        if (!lateral_fov_.has_value()) {
          ROS_ERROR("No lateral_fov specified. Cannot set sensor model.");
          break;
        }
        if (!lateral_overlap_.has_value()) {
          ROS_ERROR("No lateral_overlap specified. Cannot set sensor model.");
          break;
        }
        if (!altitude_.has_value()) {
          ROS_WARN("No altitude specified. Creating default altitude of 1m.");
          altitude_ = std::make_optional(1.0);
        } else if (altitude_.value() < std::numeric_limits<double>::epsilon()) {
          ROS_WARN(
              "Altitude to small %.3fm. Please set altitude through first "
              "z-value of polygon. Setting default altitude of 1m.",
              altitude_.value());
          altitude_ = std::make_optional(1.0);
        }
        sensor_model_ = std::make_shared<Frustum>(
            altitude_.value(), lateral_fov_.value(), lateral_overlap_.value());
        ROS_INFO("Sensor model: frustum");
        ROS_INFO_STREAM("Lateral FOV: " << lateral_fov_.value());
        ROS_INFO_STREAM("Altitude: " << altitude_.value());
        break;
      }
    }
    ROS_INFO_STREAM("Lateral overlap: " << lateral_overlap_.value());
    ROS_INFO_STREAM("Sweep distance: " << sensor_model_->getSweepDistance());
  }

  // The library object that actually does planning.
  std::unique_ptr<Planner> planner_;

  // System Parameters
  std::shared_ptr<SensorModelBase> sensor_model_;
  DecompositionType decomposition_type_;
  SensorModelType sensor_model_type_;
  bool offset_polygons_;
  bool sweep_single_direction_;
  std::optional<double> lateral_footprint_;
  std::optional<double> lateral_overlap_;
  std::optional<double> lateral_fov_;
};
}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_ROS_COVERAGE_PLANNER_H_
