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

#include "polygon_coverage_ros/polygon_planner_base.h"
#include "polygon_coverage_ros/ros_interface.h"

#include <functional>

#include <polygon_coverage_msgs/msg_from_xml_rpc.h>
#include <polygon_coverage_planners/cost_functions/path_cost_functions.h>

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

namespace polygon_coverage_planning {

PolygonPlannerBase::PolygonPlannerBase(const ros::NodeHandle& nh,
                                       const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      wall_distance_(0.0),
      path_cost_function_(
          {std::bind(&computeEuclideanPathCost, std::placeholders::_1),
           CostFunctionType::kDistance}),
      latch_topics_(true),
      global_frame_id_("world"),
      publish_plan_on_planning_complete_(false),
      publish_visualization_on_planning_complete_(true),
      set_start_goal_from_rviz_(false),
      set_polygon_from_rviz_(true),
      planning_complete_(false) {
  // Initial interactions with ROS
  getParametersFromRos();
  advertiseTopics();

  // Publish RVIZ.
  publishVisualization();
}

void PolygonPlannerBase::advertiseTopics() {
  // Advertising the visualization and planning messages
  marker_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
      "path_markers", 1, true);
  waypoint_list_pub_ = nh_.advertise<geometry_msgs::PoseArray>(
      "waypoint_list", 1, latch_topics_);
  // Services for generating the plan.
  set_polygon_srv_ = nh_private_.advertiseService(
      "set_polygon", &PolygonPlannerBase::setPolygonCallback, this);
  plan_path_srv_ = nh_private_.advertiseService(
      "plan_path", &PolygonPlannerBase::planPathCallback, this);
  // Services for performing publishing and visualization
  publish_all_srv_ = nh_private_.advertiseService(
      "publish_all", &PolygonPlannerBase::publishAllCallback, this);
  publish_visualization_srv_ = nh_private_.advertiseService(
      "publish_visualization",
      &PolygonPlannerBase::publishVisualizationCallback, this);
  publish_plan_points_srv_ = nh_private_.advertiseService(
      "publish_path_points",
      &PolygonPlannerBase::publishTrajectoryPointsCallback, this);
  // Subscribe
  clicked_point_sub_ = nh_.subscribe(
      "/clicked_point", 1, &PolygonPlannerBase::clickPointCallback, this);
  polygon_sub_ = nh_.subscribe("/polygon", 1,
                               &PolygonPlannerBase::clickPolygonCallback, this);
}

void PolygonPlannerBase::getParametersFromRos() {
  // Load the polygon from polygon message from parameter server.
  // The altitude and the global frame ID are set from the same message.
  XmlRpc::XmlRpcValue polygon_xml_rpc;
  const std::string polygon_param_name = "polygon";
  if (nh_private_.getParam(polygon_param_name, polygon_xml_rpc)) {
    polygon_coverage_msgs::PolygonWithHolesStamped poly_msg;
    if (PolygonWithHolesStampedMsgFromXmlRpc(polygon_xml_rpc, &poly_msg)) {
      PolygonWithHoles temp_pwh;
      double temp_alt;
      if (polygonFromMsg(poly_msg, &temp_pwh, &temp_alt, &global_frame_id_)) {
        ROS_INFO_STREAM("Successfully loaded polygon.");
        ROS_INFO_STREAM("Altitude: " << temp_alt << " m");
        ROS_INFO_STREAM("Global frame: " << global_frame_id_);
        ROS_INFO_STREAM("Polygon:" << temp_pwh);
        polygon_ = std::make_optional(temp_pwh);
        altitude_ = std::make_optional(temp_alt);
      }
    } else {
      ROS_WARN_STREAM("Failed reading polygon message from parameter server.");
    }
  } else {
    ROS_WARN_STREAM(
        "No polygon file specified to parameter "
        "server (parameter \""
        << polygon_param_name
        << "\"). Expecting "
           "polygon from service call.");
  }

  // Getting control params from the server
  if (!nh_private_.getParam("wall_distance", wall_distance_)) {
    ROS_WARN_STREAM("No wall distance specified. Using default value of: "
                    << wall_distance_);
  } else
    ROS_INFO_STREAM("Wall distance: " << wall_distance_ << " m");

  // Cost function
  double temp_v_max;
  if (nh_private_.getParam("v_max", temp_v_max)) {
    v_max_ = std::make_optional(temp_v_max);
  }
  double temp_a_max;
  if (nh_private_.getParam("a_max", temp_a_max)) {
    a_max_ = std::make_optional(temp_a_max);
  }

  // Cost function type.
  int cost_function_type_int = static_cast<int>(path_cost_function_.second);
  if (!nh_private_.getParam("cost_function_type", cost_function_type_int)) {
    ROS_WARN_STREAM("No cost_function_type specified. Using default value of: "
                    << path_cost_function_.second << "("
                    << getCostFunctionTypeName(path_cost_function_.second)
                    << ").");
  }
  if (!checkCostFunctionTypeValid(cost_function_type_int)) {
    ROS_WARN_STREAM("cost_function_type not valid. Resetting to default: "
                    << path_cost_function_.second << "("
                    << getCostFunctionTypeName(path_cost_function_.second)
                    << ").");
    cost_function_type_int = static_cast<int>(path_cost_function_.second);
  }
  path_cost_function_.second =
      static_cast<CostFunctionType>(cost_function_type_int);

  ROS_INFO_STREAM(
      "Cost function: " << getCostFunctionTypeName(path_cost_function_.second));
  if (path_cost_function_.second == CostFunctionType::kTime) {
    if (!v_max_.has_value() || !a_max_.has_value()) {
      ROS_WARN_COND(!v_max_.has_value(), "Velocity 'v_max' not set.");
      ROS_WARN_COND(!a_max_.has_value(), "Acceleration 'a_max' not set.");
      ROS_WARN("Falling back to distance cost function.");
      path_cost_function_.second = CostFunctionType::kDistance;
    } else {
      ROS_INFO_STREAM("v_max: " << v_max_.value()
                                << ", a_max: " << a_max_.value());
    }
  }

  switch (path_cost_function_.second) {
    case CostFunctionType::kDistance: {
      path_cost_function_.first =
          std::bind(&computeEuclideanPathCost, std::placeholders::_1);
      break;
    }
    case CostFunctionType::kTime: {
      path_cost_function_.first =
          std::bind(&computeVelocityRampPathCost, std::placeholders::_1,
                    v_max_.value(), a_max_.value());
      break;
    }
    case CostFunctionType::kWaypoints: {
      path_cost_function_.first =
          std::bind(&computeWaypointsPathCost, std::placeholders::_1);
      break;
    }
    default: {
      ROS_WARN_STREAM("Cost function type: "
                      << getCostFunctionTypeName(path_cost_function_.second)
                      << "not implemented. Using euclidean distance.");
      break;
    }
  }

  // Getting the behaviour flags
  nh_private_.getParam("latch_topics", latch_topics_);
  nh_private_.getParam("publish_plan_on_planning_complete",
                       publish_plan_on_planning_complete_);
  nh_private_.getParam("publish_visualization_on_planning_complete",
                       publish_visualization_on_planning_complete_);
  nh_private_.getParam("global_frame_id", global_frame_id_);
  nh_private_.getParam("set_start_goal_from_rviz", set_start_goal_from_rviz_);
  nh_private_.getParam("set_polygon_from_rviz", set_polygon_from_rviz_);
}

void PolygonPlannerBase::solve(const Point_2& start, const Point_2& goal) {
  ROS_INFO_STREAM("Start solving.");
  if ((planning_complete_ = solvePlanner(start, goal))) {
    ROS_INFO_STREAM("Finished plan."
                    << std::endl
                    << "Optimization Criterion: "
                    << getCostFunctionTypeName(path_cost_function_.second)
                    << std::endl
                    << "Number of waypoints: " << solution_.size() << std::endl
                    << "Start point: " << start << std::endl
                    << "Goal point: " << goal << std::endl
                    << "Altitude: " << altitude_.value() << " [m]" << std::endl
                    << "Path length: " << computeEuclideanPathCost(solution_)
                    << " [m]");
    if (v_max_.has_value() && a_max_.has_value())
      ROS_INFO_STREAM("Path time: "
                      << computeVelocityRampPathCost(solution_, v_max_.value(),
                                                     a_max_.value())
                      << " [s]");

    // Publishing the plan if requested
    if (publish_plan_on_planning_complete_) {
      publishTrajectoryPoints();
    }
    // Publishing the visualization if requested
    if (publish_visualization_on_planning_complete_) {
      publishVisualization();
    }
  } else {
    ROS_ERROR_STREAM("Failed calculating plan.");
  }
}

bool PolygonPlannerBase::publishVisualization() {
  ROS_INFO_STREAM("Sending visualization messages.");

  // Delete old markers.
  for (visualization_msgs::Marker& m : markers_.markers)
    m.action = visualization_msgs::Marker::DELETE;
  marker_pub_.publish(markers_);

  // Create new markers.
  markers_.markers.clear();
  // The planned path:
  visualization_msgs::Marker path_points, path_line_strips;
  const double kPathLineSize = 0.2;
  const double kPathPointSize = 0.2;
  if (!altitude_.has_value()) {
    ROS_WARN_STREAM("Cannot send visualization because altitude not set.");
    return false;
  }

  if (!planning_complete_) {
    ROS_WARN_STREAM(
        "Cannot send solution visualization because plan has not been made.");
  } else {
    createMarkers(solution_, altitude_.value(), global_frame_id_,
                  "vertices_and_strip", Color::Gray(), Color::Gray(),
                  kPathLineSize, kPathPointSize, &path_points,
                  &path_line_strips);
    markers_.markers.push_back(path_points);
    markers_.markers.push_back(path_line_strips);

    // Start and end points
    visualization_msgs::Marker start_point, end_point;
    createStartAndEndPointMarkers(solution_.front(), solution_.back(),
                                  altitude_.value(), global_frame_id_, "points",
                                  &start_point, &end_point);
    markers_.markers.push_back(start_point);
    markers_.markers.push_back(end_point);

    // Start and end text.
    visualization_msgs::Marker start_text, end_text;
    createStartAndEndTextMarkers(solution_.front(), solution_.back(),
                                 altitude_.value(), global_frame_id_, "points",
                                 &start_text, &end_text);
    markers_.markers.push_back(start_text);
    markers_.markers.push_back(end_text);
  }

  // The original polygon:
  const double kPolygonLineSize = 0.4;
  visualization_msgs::MarkerArray polygon;
  if (!polygon_.has_value()) {
    ROS_WARN_STREAM("Cannot send visualization because polygon not set.");
    return false;
  }
  createPolygonMarkers(polygon_.value(), altitude_.value(), global_frame_id_,
                       "polygon", Color::Black(), Color::Black(),
                       kPolygonLineSize, kPolygonLineSize, &polygon);
  markers_.markers.insert(markers_.markers.end(), polygon.markers.begin(),
                          polygon.markers.end());

  // The decomposed polygons.
  visualization_msgs::MarkerArray decomposition_markers =
      createDecompositionMarkers();
  markers_.markers.insert(markers_.markers.end(),
                          decomposition_markers.markers.begin(),
                          decomposition_markers.markers.end());

  // Publishing
  marker_pub_.publish(markers_);

  // Success
  return true;
}

bool PolygonPlannerBase::publishTrajectoryPoints() {
  if (!planning_complete_) {
    ROS_WARN("Cannot send trajectory messages because plan has not been made.");
    return false;
  }
  ROS_INFO_STREAM("Sending trajectory messages");

  // Convert path to pose array.
  geometry_msgs::PoseArray trajectory_points_pose_array;
  if (!altitude_.has_value()) {
    ROS_WARN_STREAM("Cannot send trajectory because altitude not set.");
    return false;
  }
  poseArrayMsgFromPath(solution_, altitude_.value(), global_frame_id_,
                       &trajectory_points_pose_array);

  // Publishing
  waypoint_list_pub_.publish(trajectory_points_pose_array);

  // Success
  return true;
}

bool PolygonPlannerBase::setPolygonCallback(
    polygon_coverage_msgs::PolygonService::Request& request,
    polygon_coverage_msgs::PolygonService::Response& response) {
  PolygonWithHoles temp_pwh;
  double temp_alt;
  if (!polygonFromMsg(request.polygon, &temp_pwh, &temp_alt,
                      &global_frame_id_)) {
    ROS_ERROR_STREAM("Failed loading correct polygon.");
    ROS_ERROR_STREAM("Planner is in an invalid state.");
    polygon_.reset();
    return false;
  }
  polygon_ = std::make_optional(temp_pwh);
  altitude_ = std::make_optional(temp_alt);

  ROS_INFO_STREAM("Successfully loaded polygon.");
  ROS_INFO_STREAM("Altitude: " << altitude_.value() << "m");
  ROS_INFO_STREAM("Global frame: " << global_frame_id_);
  ROS_INFO_STREAM("Polygon:" << polygon_.value());

  response.success = resetPlanner();
  return true;  // Still return true to identify service has been reached.
}

bool PolygonPlannerBase::planPathCallback(
    polygon_coverage_msgs::PlannerService::Request& request,
    polygon_coverage_msgs::PlannerService::Response& response) {
  planning_complete_ = false;
  if (!polygon_.has_value()) {
    ROS_WARN("Polygon not set. Cannot plan path.");
    response.success = planning_complete_;
    return true;
  }
  const Point_2 start(request.start_pose.pose.position.x,
                      request.start_pose.pose.position.y);
  const Point_2 goal(request.goal_pose.pose.position.x,
                     request.goal_pose.pose.position.y);
  solve(start, goal);  // Calculate optimal path.
  if (altitude_.has_value()) {
    msgMultiDofJointTrajectoryFromPath(solution_, altitude_.value(),
                                       &response.sampled_plan);
  } else {
    ROS_WARN("Cannot plan path. Altitude not set.");
  }
  response.success = planning_complete_;
  return true;
}

bool PolygonPlannerBase::publishAllCallback(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  bool success_publish_trajectory = publishTrajectoryPoints();
  bool success_publish_visualization = publishVisualization();
  return (success_publish_trajectory && success_publish_visualization);
}

bool PolygonPlannerBase::publishVisualizationCallback(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  return publishVisualization();
}

bool PolygonPlannerBase::publishTrajectoryPointsCallback(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  return publishTrajectoryPoints();
}

// Reset the planner when a new polygon is set.
bool PolygonPlannerBase::resetPlanner() {
  ROS_ERROR_STREAM("resetPlanner is not implemented.");
  return false;
}

void PolygonPlannerBase::clickPointCallback(
    const geometry_msgs::PointStampedConstPtr& msg) {
  if (!set_start_goal_from_rviz_) return;

  if (!start_.has_value()) {
    ROS_INFO("Selecting START from RVIZ PublishPoint tool.");
    start_ = std::make_optional<Point_2>(msg->point.x, msg->point.y);
  } else if (!goal_.has_value()) {
    ROS_INFO("Selecting GOAL from RVIZ PublishPoint tool.");
    goal_ = std::make_optional<Point_2>(msg->point.x, msg->point.y);
  }

  if (start_.has_value() && goal_.has_value()) {
    solve(start_.value(), goal_.value());
    start_.reset();
    goal_.reset();
  }

  return;
}

void PolygonPlannerBase::clickPolygonCallback(
    const polygon_coverage_msgs::PolygonWithHolesStamped& msg) {
  if (!set_polygon_from_rviz_) return;

  ROS_INFO("Updating polygon from RVIZ polygon tool.");
  PolygonWithHoles temp_pwh;
  double temp_alt;
  if (polygonFromMsg(msg, &temp_pwh, &temp_alt, &global_frame_id_)) {
    ROS_INFO_STREAM("Successfully loaded polygon.");
    ROS_INFO_STREAM("Altitude: " << temp_alt << " m");
    ROS_INFO_STREAM("Global frame: " << global_frame_id_);
    ROS_INFO_STREAM("Polygon:" << temp_pwh);
    polygon_ = std::make_optional(temp_pwh);
    altitude_ = std::make_optional(temp_alt);
  }

  planning_complete_ = false;
  resetPlanner();
  publishVisualization();

  return;
}

}  // namespace polygon_coverage_planning
