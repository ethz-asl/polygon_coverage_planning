#include "mav_coverage_planning_ros/nodes/2d/base_planner_2d.h"

#include <functional>

#include <mav_2d_coverage_planning/cost_functions/path_cost_functions.h>
#include <mav_coverage_planning_ros/conversions/msg_from_xml_rpc.h>
#include <mav_coverage_planning_ros/conversions/ros_interface.h>

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

namespace mav_coverage_planning {

constexpr double kThrottleRate = 1.0 / 10.0;

BasePlanner2D::Settings::Settings()
    : sweep_cost_function(
          std::bind(&computeEuclideanPathCost, std::placeholders::_1)),
      altitude(-1.0),
      latch_topics(true),
      local_frame_id("odom"),
      global_frame_id("world"),
      publish_plan_on_planning_complete(false),
      publish_visualization_on_planning_complete(true),
      robot_size(1.0),
      wall_dist(0.0),
      min_view_overlap(0.0),
      sweep_around_obstacles(false),
      decomposition_type("convex")
      {}

BasePlanner2D::BasePlanner2D(const ros::NodeHandle& nh,
                             const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      planning_complete_(false),
      odometry_set_(false),
      odometry_in_global_frame_(true) {
  // Initial interactions with ROS
  getParametersFromRos();
  subscribeToTopics();
  advertiseTopics();
}

void BasePlanner2D::subscribeToTopics() {
  odometry_sub_ = nh_.subscribe("odometry", 1,
                                &BasePlanner2D::receiveOdometryCallback, this);
  T_G_L_sub_ =
      nh_.subscribe("T_G_L", 1, &BasePlanner2D::receiveTransformCallback, this);
}

void BasePlanner2D::advertiseTopics() {
  // Advertising the visualization and planning messages
  marker_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
      "path_markers", 1, settings_.latch_topics);
  waypoint_list_pub_ = nh_.advertise<geometry_msgs::PoseArray>(
      "waypoint_list", 1, settings_.latch_topics);
  // Services for generating the plan.
  set_polygon_srv_ = nh_private_.advertiseService(
      "set_polygon", &BasePlanner2D::setPolygonCallback, this);
  plan_path_srv_ = nh_private_.advertiseService(
      "plan_path", &BasePlanner2D::planPathCallback, this);
  plan_path_from_and_to_odometry_srv_ = nh_private_.advertiseService(
      "plan_path_from_and_to_odometry",
      &BasePlanner2D::planPathFromAndToOdometryCallback, this);
  plan_path_from_odometry_to_goal_srv_ = nh_private_.advertiseService(
      "plan_path_from_odometry_to_goal",
      &BasePlanner2D::planPathFromOdometryToGoalCallback, this);
  // Services for performing publishing and visualization
  publish_all_srv_ = nh_private_.advertiseService(
      "publish_all", &BasePlanner2D::publishAllCallback, this);
  publish_visualization_srv_ = nh_private_.advertiseService(
      "publish_visualization", &BasePlanner2D::publishVisualizationCallback,
      this);
  publish_plan_points_srv_ = nh_private_.advertiseService(
      "publish_path_points", &BasePlanner2D::publishTrajectoryPointsCallback,
      this);
}

void BasePlanner2D::getParametersFromRos() {
  // Getting control params from the server
  if (!nh_private_.getParam("robot_size", settings_.robot_size)) {
    ROS_WARN_STREAM("No robot size specified. Using default value of: "
                    << settings_.robot_size);
  }
  if (!nh_private_.getParam("wall_dist", settings_.wall_dist)) {
  }
  if (!nh_private_.getParam("decomposition_type", settings_.decomposition_type)) {
  }
  if (!nh_private_.getParam("sweep_around_obstacles", settings_.sweep_around_obstacles)) {
  }
  if (!nh_private_.getParam("min_view_overlap", settings_.min_view_overlap)) {
  }
  
  if (!nh_private_.getParam("local_frame_id", settings_.local_frame_id)) {
    ROS_WARN_STREAM("No local frame id specified. Using default value of: "
                    << settings_.local_frame_id);
  }

  // Cost function
  nh_private_.param("v_max", settings_.v_max, kDefaultVMax);
  nh_private_.param("a_max", settings_.a_max, kDefaultAMax);

  int cost_function_type_int = static_cast<int>(settings_.cost_function_type);
  if (!nh_private_.getParam("cost_function_type", cost_function_type_int)) {
    ROS_WARN_STREAM("No cost_function_type specified. Using default value of: "
                    << settings_.cost_function_type << "("
                    << settings_.getCostFunctionTypeName() << ").");
  }
  settings_.cost_function_type =
      static_cast<Settings::CostFunctionType>(cost_function_type_int);
  if (!settings_.checkCostFunctionTypeValid()) {
    settings_.cost_function_type = Settings::CostFunctionType::kDistance;
    ROS_WARN_STREAM("cost_function_type not valid. Resetting to default: "
                    << settings_.cost_function_type << "("
                    << settings_.getCostFunctionTypeName() << ").");
  }

  switch (settings_.cost_function_type) {
    case Settings::CostFunctionType::kDistance: {
      settings_.sweep_cost_function =
          std::bind(&computeEuclideanPathCost, std::placeholders::_1);
      break;
    }
    case Settings::CostFunctionType::kTime: {
      settings_.sweep_cost_function =
          std::bind(&computeVelocityRampPathCost, std::placeholders::_1,
                    settings_.v_max, settings_.a_max);
      break;
    }
    case Settings::CostFunctionType::kWaypoints: {
      settings_.sweep_cost_function =
          std::bind(&computeWaypointsPathCost, std::placeholders::_1);
      break;
    }
    default: {
      ROS_WARN_STREAM("Cost function type: "
                      << settings_.getCostFunctionTypeName()
                      << "not implemented. Using euclidean distance.");
      break;
    }
  }

  // Load the polygon from polygon message from parameter server.
  // The altitude and the global frame ID are set from the same message.
  XmlRpc::XmlRpcValue polygon_xml_rpc;
  const std::string polygon_param_name = "polygon";
  if (nh_private_.getParam(polygon_param_name, polygon_xml_rpc)) {
    mav_planning_msgs::PolygonWithHolesStamped poly_msg;
    if (PolygonWithHolesStampedMsgFromXmlRpc(polygon_xml_rpc, &poly_msg)) {
      if (polygonFromMsg(poly_msg, &settings_.polygon, &settings_.altitude,
                         &settings_.global_frame_id)) {
        ROS_INFO_STREAM("Successfully loaded polygon.");
        ROS_INFO_STREAM("Altiude: " << settings_.altitude << "m");
        ROS_INFO_STREAM("Global frame: " << settings_.global_frame_id);
        ROS_INFO_STREAM("Polygon:" << settings_.polygon);
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

  // Getting the behaviour flags
  nh_private_.getParam("latch_topics", settings_.latch_topics);
  nh_private_.getParam("publish_plan_on_planning_complete",
                       settings_.publish_plan_on_planning_complete);
  nh_private_.getParam("publish_visualization_on_planning_complete",
                       settings_.publish_visualization_on_planning_complete);
}

void BasePlanner2D::receiveOdometryCallback(const nav_msgs::Odometry& msg) {
  mav_msgs::eigenOdometryFromMsg(msg, &odometry_);
  odometry_set_ = true;
  ROS_INFO_STREAM_ONCE("Received first odometry message.");

  odometry_in_global_frame_ =
      (msg.header.frame_id == settings_.global_frame_id);
  if (!odometry_in_global_frame_) {
    ROS_INFO_STREAM_THROTTLE(kThrottleRate,
                             "Odometry message in frame: \""
                                 << msg.header.frame_id
                                 << "\". Will convert it using T_G_L.");
  }
}

void BasePlanner2D::receiveTransformCallback(
    const geometry_msgs::TransformStamped& msg) {
  tf::transformMsgToKindr(msg.transform, &T_G_L_);
  if (msg.header.frame_id != settings_.global_frame_id ||
      msg.child_frame_id != settings_.local_frame_id) {
    ROS_WARN_STREAM_ONCE(
        "Expected and received T_G_L frame ids do "
        "not agree. Expected: G = \""
        << settings_.global_frame_id << "\", L = \"" << settings_.local_frame_id
        << "\" Received: G = \"" << msg.header.frame_id << "\", L = \""
        << msg.child_frame_id << "\".");
  }
}

void BasePlanner2D::solve(const Point_2& start, const Point_2& goal) {
  ROS_INFO_STREAM("Start solving.");
  if ((planning_complete_ = solvePlanner(start, goal))) {
    ROS_INFO_STREAM("Finished plan."
                    << std::endl
                    << "Optimization Criterion: "
                    << settings_.getCostFunctionTypeName() << std::endl
                    << "Number of waypoints: " << solution_.size() << std::endl
                    << "Start point: " << start << std::endl
                    << "Goal point: " << goal << std::endl
                    << "Altitude: " << settings_.altitude << " [m]" << std::endl
                    << "Path length: " << computeEuclideanPathCost(solution_)
                    << " [m]" << std::endl
                    << "Path time: "
                    << computeVelocityRampPathCost(solution_, settings_.v_max,
                                                   settings_.a_max)
                    << " [s]");
    // Publishing the plan if requested
    if (settings_.publish_plan_on_planning_complete) {
      publishTrajectoryPoints();
    }
    // Publishing the visualization if requested
    if (settings_.publish_visualization_on_planning_complete) {
      publishVisualization();
    }
  } else {
    ROS_ERROR_STREAM("Failed calculating plan.");
    visualization_msgs::MarkerArray markers;
  }
}

bool BasePlanner2D::publishVisualization() {
  if (!planning_complete_) {
    ROS_WARN_STREAM(
        "Cannot send visualization message because plan "
        "hasn't been made yet.");
    return false;
  }
  ROS_INFO_STREAM("Sending visualization messages.");

  // Creating the marker array
  visualization_msgs::MarkerArray markers;

  // The planned path:
  visualization_msgs::Marker path_points, path_line_strips;
  createMarkers(solution_, settings_.altitude, settings_.global_frame_id,
                "vertices_and_strip", mav_visualization::Color::Red(),
                mav_visualization::Color::Green(), &path_points,
                &path_line_strips);
  markers.markers.push_back(path_points);
  markers.markers.push_back(path_line_strips);

  // Start and end points
  visualization_msgs::Marker start_point, end_point;
  createStartAndEndPointMarkers(solution_.front(), solution_.back(),
                                settings_.altitude, settings_.global_frame_id,
                                "points", &start_point, &end_point);
  markers.markers.push_back(start_point);
  markers.markers.push_back(end_point);

  // The original polygon:
  visualization_msgs::MarkerArray polygon;
  createPolygonMarkers(settings_.polygon, settings_.altitude,
                       settings_.global_frame_id, "polygon",
                       mav_visualization::Color::Blue(),
                       mav_visualization::Color::Orange(), &polygon);
  markers.markers.insert(markers.markers.end(), polygon.markers.begin(),
                         polygon.markers.end());

  // The decomposed polygons.
  std::vector<Polygon> convex_decomposition;
  if (settings_.decomposition_type.compare("convex") == 0) {
    settings_.polygon.computeConvexDecompositionFromPolygonWithHoles(
      &convex_decomposition);
  } else if (settings_.decomposition_type.compare("bcd") == 0) {
    settings_.polygon.computeBCDFromPolygonWithHoles(
      &convex_decomposition);
  }
  for (size_t i = 0; i < convex_decomposition.size(); ++i) {
    visualization_msgs::MarkerArray convex_polygon_markers;
    std::string name = "convex_polygon_" + std::to_string(i);
    createPolygonMarkers(
        convex_decomposition[i], settings_.altitude, settings_.global_frame_id,
        name, mav_visualization::Color::Red(), mav_visualization::Color::Red(),
        &convex_polygon_markers);
    markers.markers.insert(markers.markers.end(),
                           convex_polygon_markers.markers.begin(),
                           convex_polygon_markers.markers.end());
  }

  // Publishing
  marker_pub_.publish(markers);

  // Success
  return true;
}

bool BasePlanner2D::publishTrajectoryPoints() {
  if (!planning_complete_) {
    ROS_WARN(
        "Cannot send trajectory messages because plan hasn\'t been made, yet.");
    return false;
  }
  ROS_INFO_STREAM("Sending trajectory messages");

  // Convert path to pose array.
  geometry_msgs::PoseArray trajectory_points_pose_array;
  poseArrayMsgFromPath(solution_, settings_.altitude, settings_.global_frame_id,
                       &trajectory_points_pose_array);

  // Publishing
  waypoint_list_pub_.publish(trajectory_points_pose_array);

  // Success
  return true;
}

bool BasePlanner2D::setPolygonCallback(
    mav_planning_msgs::PolygonService::Request& request,
    mav_planning_msgs::PolygonService::Response& response) {
  planning_complete_ = false;

  if (!polygonFromMsg(request.polygon, &settings_.polygon, &settings_.altitude,
                      &settings_.global_frame_id)) {
    ROS_ERROR_STREAM("Failed loading correct polygon.");
    ROS_ERROR_STREAM("Planner is in an invalid state.");
    settings_.polygon = Polygon();
  }
  response.success = resetPlanner();
  return true;  // Still return true to identify service has been reached.
}

bool BasePlanner2D::planPathCallback(
    mav_planning_msgs::PlannerService::Request& request,
    mav_planning_msgs::PlannerService::Response& response) {
  const Point_2 start(request.start_pose.pose.position.x,
                      request.start_pose.pose.position.y);
  const Point_2 goal(request.goal_pose.pose.position.x,
                     request.goal_pose.pose.position.y);
  solve(start, goal);  // Calculate optimal path.
  msgMultiDofJointTrajectoryFromPath(solution_, settings_.altitude,
                                     &response.sampled_plan);
  response.success = planning_complete_;
  return true;
}

bool BasePlanner2D::planningRequestStartPoseFromOdometry(
    mav_planning_msgs::PlannerService::Request* req) const {
  if (!odometry_set_) {
    ROS_ERROR_STREAM("Did not receive odometry.");
    return false;
  }
  // Convert odometry to global frame id.
  mav_msgs::EigenOdometry odometry_global =
      globalOdometryFromOdometry(odometry_);
  req->start_pose.pose.position.x = odometry_global.position_W.x();
  req->start_pose.pose.position.y = odometry_global.position_W.y();
  return true;
}

bool BasePlanner2D::planPathFromAndToOdometryCallback(
    mav_planning_msgs::PlannerService::Request& request,
    mav_planning_msgs::PlannerService::Response& response) {
  // Convert odometry msg to planning request.
  if (planningRequestStartPoseFromOdometry(&request)) {
    request.goal_pose = request.start_pose;
    planPathCallback(request, response);
  } else {
    response.success = false;
  }
  return true;
}

bool BasePlanner2D::planPathFromOdometryToGoalCallback(
    mav_planning_msgs::PlannerService::Request& request,
    mav_planning_msgs::PlannerService::Response& response) {
  // Convert odometry msg to planning request.
  if (planningRequestStartPoseFromOdometry(&request)) {
    planPathCallback(request, response);
  } else {
    response.success = false;
  }
  return true;
}

bool BasePlanner2D::publishAllCallback(std_srvs::Empty::Request& request,
                                       std_srvs::Empty::Response& response) {
  bool success_publish_trajectory = publishTrajectoryPoints();
  bool success_publish_visualization = publishVisualization();
  return (success_publish_trajectory && success_publish_visualization);
}

bool BasePlanner2D::publishVisualizationCallback(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  return publishVisualization();
}

bool BasePlanner2D::publishTrajectoryPointsCallback(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  return publishTrajectoryPoints();
}

mav_msgs::EigenOdometry BasePlanner2D::globalOdometryFromOdometry(
    const mav_msgs::EigenOdometry& odometry) const {
  // Check if odometry is already in local frame.
  if (odometry_in_global_frame_) {
    return odometry;
  } else {
    ROS_INFO_STREAM(
        "Transforming odometry message from local frame using T_G_L:\n"
        << T_G_L_);
    mav_msgs::EigenOdometry odometry_global;
    odometry_global.position_W = T_G_L_ * odometry.position_W;
    odometry_global.orientation_W_B =
        T_G_L_.getRotation().toImplementation() * odometry.orientation_W_B;
    return odometry_global;
  }
}

// Reset the planner when a new polygon is set.
bool BasePlanner2D::resetPlanner() {
  ROS_ERROR_STREAM("resetPlanner is not implemented.");
  return false;
}

}  // namespace mav_coverage_planning
