#include "mav_coverage_planning/base_planner_ros.h"

#include <glog/logging.h>

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <planning_msgs/PolygonWithHolesStamped.h>

#include "mav_coverage_planning/msg_from_xml_rpc.h"
#include "mav_coverage_planning/ros_interface.h"

namespace mav_coverage_planning {

constexpr double kThrottleRate = 1.0 / 10.0;

BasePlannerRos::BasePlannerRos(const ros::NodeHandle& nh,
                               const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      solution_(),
      planning_complete_(false),
      odometry_set_(false),
      odometry_in_global_frame_(true),
      global_frame_id_(""),  // Set with polygon.
      local_frame_id_(kDefaultLocalFrameID),
      altitude_(-1.0),  // Set with polygon.
      cost_function_(kDefaultCostFunctionType, kDefaultMaximumVelocity,
                     kDefaultMaximumAcceleration),
      latch_topic_(kDefaultLatchTopic),
      output_prefix_(kDefaultOutputPrefix),
      publish_plan_points_on_planning_complete_(
          kDefaultPublishPlanPointsOnPlanningComplete),
      publish_visualization_on_planning_complete_(
          kDefaultPublishVisualizationOnPlanningComplete) {
  T_G_L_.setIdentity();

  // Initial interactions with ROS
  getParametersFromRos();
  subscribeToTopics();
  advertiseTopics();
}

void BasePlannerRos::subscribeToTopics() {
  odometry_sub_ = nh_.subscribe("odometry", 1,
                                &BasePlannerRos::receiveOdometryCallback, this);
  T_G_L_sub_ = nh_.subscribe("T_G_L", 1,
                             &BasePlannerRos::receiveTransformCallback, this);
}

void BasePlannerRos::advertiseTopics() {
  // Advertising the visualization and planning messages
  marker_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>(
      "path_markers", 1, latch_topic_);
  waypoint_list_pub_ =
      nh_.advertise<geometry_msgs::PoseArray>("waypoint_list", 1, latch_topic_);
  // Services for generating the plan.
  set_polygon_srv_ = nh_private_.advertiseService(
      "set_polygon", &BasePlannerRos::setPolygonCallback, this);
  plan_path_srv_ = nh_private_.advertiseService(
      "plan_path", &BasePlannerRos::planPathCallback, this);
  plan_path_from_and_to_odometry_srv_ = nh_private_.advertiseService(
      "plan_path_from_and_to_odometry",
      &BasePlannerRos::planPathFromAndToOdometryCallback, this);
  plan_path_from_odometry_to_goal_srv_ = nh_private_.advertiseService(
      "plan_path_from_odometry_to_goal",
      &BasePlannerRos::planPathFromOdometryToGoalCallback, this);
  // Services for performing publishing and visualization
  publish_all_srv_ = nh_private_.advertiseService(
      "publish_all", &BasePlannerRos::publishAllCallback, this);
  publish_visualization_srv_ = nh_private_.advertiseService(
      "publish_visualization", &BasePlannerRos::publishVisualizationCallback,
      this);
  publish_plan_points_srv_ = nh_private_.advertiseService(
      "publish_path_points", &BasePlannerRos::publishTrajectoryPointsCallback,
      this);
}

void BasePlannerRos::getParametersFromRos() {
  // Getting control params from the server
  if (!nh_private_.getParam("local_frame_id", local_frame_id_)) {
    ROS_WARN_STREAM(output_prefix_
                    << "No local frame id specified. Using default value of: "
                    << local_frame_id_);
  }

  // Cost function
  int cost_function_type_int = static_cast<int>(cost_function_.getType());
  if (!nh_private_.getParam("cost_function_type", cost_function_type_int)) {
    ROS_WARN_STREAM(
        output_prefix_
        << "No cost_function_type specified. Using default value of: "
        << cost_function_.getType() << "("
        << CostFunction::getTypeName(cost_function_.getType()) << ").");
  }
  CostFunction::Type cost_function_type =
      static_cast<CostFunction::Type>(cost_function_type_int);
  if (!CostFunction::checkTypeValid(cost_function_type)) {
    cost_function_type = kDefaultCostFunctionType;
    ROS_WARN_STREAM(output_prefix_
                    << "cost_function_type not valid. Resetting to default: "
                    << cost_function_type << "("
                    << CostFunction::getTypeName(cost_function_type) << ").");
  }

  double v_max = static_cast<int>(cost_function_.getVMax());
  if (!nh_private_.getParam("v_max", v_max) &&
      cost_function_type == CostFunction::Type::kVelocityRampTime) {
    ROS_WARN_STREAM(output_prefix_
                    << "No v_max specified. Using default value of: " << v_max
                    << "[m/s]");
  }

  double a_max = cost_function_.getAMax();
  if (!nh_private_.getParam("a_max", a_max) &&
      cost_function_type == CostFunction::Type::kVelocityRampTime) {
    ROS_WARN_STREAM(output_prefix_
                    << "No a_max specified. Using default value of: " << a_max
                    << "[m/s/s]");
  }
  cost_function_ = CostFunction(cost_function_type, v_max, a_max);

  // Load the polygon from polygon message from parameter server.
  // The altitude and the global frame ID are set from the same message.
  XmlRpc::XmlRpcValue polygon_xml_rpc;
  const std::string polygon_param_name = "polygon";
  if (nh_private_.getParam(polygon_param_name, polygon_xml_rpc)) {
    planning_msgs::PolygonWithHolesStamped poly_msg;
    if (PolygonWithHolesStampedMsgFromXmlRpc(polygon_xml_rpc, &poly_msg)) {
      if (polygonFromMsg(poly_msg, &polygon_, &altitude_, &global_frame_id_)) {
        ROS_INFO_STREAM(output_prefix_ << "Successfully loaded polygon.");
        ROS_INFO_STREAM(output_prefix_ << "Altiude: " << altitude_ << "m");
        ROS_INFO_STREAM(output_prefix_ << "Global frame: " << global_frame_id_);
        ROS_INFO_STREAM(output_prefix_ << "Polygon:" << polygon_);
      }
    } else {
      ROS_WARN_STREAM(
          output_prefix_
          << "Failed reading polygon message from parameter server.");
    }
  } else {
    ROS_WARN_STREAM(output_prefix_ << "No polygon file specified to parameter "
                                      "server (parameter \""
                                   << polygon_param_name
                                   << "\"). Expecting "
                                      "polygon from service call.");
  }

  // Getting the behaviour flags
  nh_private_.param("latch_topic", latch_topic_, latch_topic_);
  nh_private_.param("publish_plan_points_on_planning_complete",
                    publish_plan_points_on_planning_complete_,
                    publish_plan_points_on_planning_complete_);
  nh_private_.param("publish_visualization_on_planning_complete",
                    publish_visualization_on_planning_complete_,
                    publish_visualization_on_planning_complete_);
}

void BasePlannerRos::receiveOdometryCallback(const nav_msgs::Odometry& msg) {
  mav_msgs::eigenOdometryFromMsg(msg, &odometry_);
  odometry_set_ = true;
  ROS_INFO_STREAM_ONCE(output_prefix_ << "Received first odometry message.");

  odometry_in_global_frame_ = (msg.header.frame_id == global_frame_id_);
  if (!odometry_in_global_frame_) {
    ROS_INFO_STREAM_THROTTLE(
        kThrottleRate, output_prefix_ << "Odometry message in frame: \""
                                      << msg.header.frame_id
                                      << "\". Will convert it using T_G_L.");
  }
}

void BasePlannerRos::receiveTransformCallback(
    const geometry_msgs::TransformStamped& msg) {
  tf::transformMsgToKindr(msg.transform, &T_G_L_);
  if (msg.header.frame_id != global_frame_id_ ||
      msg.child_frame_id != local_frame_id_) {
    ROS_WARN_STREAM_ONCE(output_prefix_
                         << "Expected and received T_G_L frame ids do "
                            "not coincide. Expected: G = \""
                         << global_frame_id_ << "\", L = \"" << local_frame_id_
                         << "\" Received: G = \"" << msg.header.frame_id
                         << "\", L = \"" << msg.child_frame_id << "\".");
  }
}

void BasePlannerRos::solve(const Eigen::Vector2d& start,
                           const Eigen::Vector2d& goal) {
  ROS_INFO_STREAM(output_prefix_ << "Start solving.");
  if ((planning_complete_ = solvePlanner(start, goal))) {
    ROS_INFO_STREAM(
        output_prefix_
        << "Finished plan." << std::endl
        << "Optimization Criterion: "
        << CostFunction::getTypeName(cost_function_.getType()) << std::endl
        << "Number of waypoints: " << solution_.size() << std::endl
        << "Start point: " << start.transpose() << std::endl
        << "Goal point: " << goal.transpose() << std::endl
        << "Altitude: " << altitude_ << " [m]" << std::endl
        << "Path length: " << cost_function_.computeCostEuclidean(solution_)
        << " [m]" << std::endl
        << "Path time: " << cost_function_.computeCostVelocityRamps(solution_)
        << " [s] assuming velocity ramp trajectories with v_max: "
        << cost_function_.getVMax()
        << " [m/s] and a_max: " << cost_function_.getAMax() << " [m/s/s].");
    // Publishing the plan if requested
    if (publish_plan_points_on_planning_complete_) {
      publishTrajectoryPoints();
    }
    // Publishing the visualization if requested
    if (publish_visualization_on_planning_complete_) {
      publishVisualization();
    }
  } else {
    ROS_ERROR_STREAM(output_prefix_ << "Failed calculating plan.");
  }
}

bool BasePlannerRos::solvePlanner(const Eigen::Vector2d& start,
                                  const Eigen::Vector2d& goal) {
  ROS_ERROR_STREAM(kOutputPrefix << "solvePlanner is not implemented.");
  return false;
}

bool BasePlannerRos::publishVisualization() {
  if (!planning_complete_) {
    ROS_WARN_STREAM(output_prefix_
                    << "Cannot send visualization message because plan "
                       "hasn't been made yet.");
    return false;
  }
  ROS_INFO_STREAM(output_prefix_ << "Sending visualization messages.");

  // Creating the marker array
  visualization_msgs::MarkerArray markers;

  // The planned path:
  visualization_msgs::Marker path_points, path_line_strips;
  createMarkers(solution_, altitude_, global_frame_id_, "vertices_and_strip",
                mav_visualization::Color::Red(),
                mav_visualization::Color::Green(), &path_points,
                &path_line_strips);
  markers.markers.push_back(path_points);
  markers.markers.push_back(path_line_strips);

  // Start and end points
  visualization_msgs::Marker start_point, end_point;
  createStartAndEndPointMarkers(solution_.front(), solution_.back(), altitude_,
                                global_frame_id_, "points", &start_point,
                                &end_point);
  markers.markers.push_back(start_point);
  markers.markers.push_back(end_point);

  // The original polygon:
  visualization_msgs::MarkerArray polygon;
  createPolygonMarkers(polygon_, altitude_, global_frame_id_, "polygon",
                       mav_visualization::Color::Blue(),
                       mav_visualization::Color::Orange(), &polygon);
  markers.markers.insert(markers.markers.end(), polygon.markers.begin(),
                         polygon.markers.end());

  // The decomposed polygons.
  std::vector<Polygon> decomposed_polygons;
  polygon_.computeConvexDecomposition(&decomposed_polygons);
  for (size_t i = 0; i < decomposed_polygons.size(); ++i) {
    visualization_msgs::MarkerArray decomposed_polygon_markers;
    std::string name = "decomposed_polygon_" + std::to_string(i);
    createPolygonMarkers(decomposed_polygons[i], altitude_, global_frame_id_,
                         name, mav_visualization::Color::Red(),
                         mav_visualization::Color::Red(),
                         &decomposed_polygon_markers);
    markers.markers.insert(markers.markers.end(),
                           decomposed_polygon_markers.markers.begin(),
                           decomposed_polygon_markers.markers.end());
  }

  // Publishing
  marker_pub_.publish(markers);

  // Success
  return true;
}

bool BasePlannerRos::publishTrajectoryPoints() {
  if (!planning_complete_) {
    ROS_WARN(
        "Cannot send trajectory messages because plan hasn't been made yet.");
    return false;
  }
  ROS_INFO_STREAM(output_prefix_ << "Sending trajectory messages");

  // Convert path to pose array.
  geometry_msgs::PoseArray trajectory_points_pose_array;
  poseArrayMsgFromPath(solution_, altitude_, global_frame_id_,
                       &trajectory_points_pose_array);

  // Publishing
  waypoint_list_pub_.publish(trajectory_points_pose_array);

  // Success
  return true;
}

bool BasePlannerRos::setPolygonCallback(
    planning_msgs::PolygonService::Request& request,
    planning_msgs::PolygonService::Response& response) {
  planning_complete_ = false;

  if (!polygonFromMsg(request.polygon, &polygon_, &altitude_,
                      &global_frame_id_)) {
    ROS_ERROR_STREAM(output_prefix_ << "Failed loading correct polygon.");
    ROS_ERROR_STREAM(output_prefix_ << "Planner is in an invalid state.");
    polygon_ = Polygon();
  }
  response.success = resetPlanner();
  return true;  // Still return true to identify service has been reached.
}

bool BasePlannerRos::planPathCallback(
    planning_msgs::PlannerService::Request& request,
    planning_msgs::PlannerService::Response& response) {
  const Eigen::Vector2d start(request.start_pose.pose.position.x,
                              request.start_pose.pose.position.y);
  const Eigen::Vector2d goal(request.goal_pose.pose.position.x,
                             request.goal_pose.pose.position.y);
  solve(start, goal);  // Calculate optimal path.
  msgMultiDofJointTrajectoryFromPath(solution_, altitude_,
                                     &response.sampled_plan);
  response.success = planning_complete_;
  return true;
}

bool BasePlannerRos::planningRequestStartPoseFromOdometry(
    planning_msgs::PlannerService::Request* req) const {
  if (!odometry_set_) {
    ROS_ERROR_STREAM(output_prefix_ << "Did not receive odometry.");
    return false;
  }
  // Convert odometry to global frame id.
  mav_msgs::EigenOdometry odometry_global =
      globalOdometryFromOdometry(odometry_);
  req->start_pose.pose.position.x = odometry_global.position_W.x();
  req->start_pose.pose.position.y = odometry_global.position_W.y();
  return true;
}

bool BasePlannerRos::planPathFromAndToOdometryCallback(
    planning_msgs::PlannerService::Request& request,
    planning_msgs::PlannerService::Response& response) {
  // Convert odometry msg to planning request.
  if (planningRequestStartPoseFromOdometry(&request)) {
    request.goal_pose = request.start_pose;
    planPathCallback(request, response);
  } else {
    response.success = false;
  }
  return true;
}

bool BasePlannerRos::planPathFromOdometryToGoalCallback(
    planning_msgs::PlannerService::Request& request,
    planning_msgs::PlannerService::Response& response) {
  // Convert odometry msg to planning request.
  if (planningRequestStartPoseFromOdometry(&request)) {
    planPathCallback(request, response);
  } else {
    response.success = false;
  }
  return true;
}

bool BasePlannerRos::publishAllCallback(std_srvs::Empty::Request& request,
                                        std_srvs::Empty::Response& response) {
  bool success_publish_trajectory = publishTrajectoryPoints();
  bool success_publish_visualization = publishVisualization();
  return (success_publish_trajectory && success_publish_visualization);
}

bool BasePlannerRos::publishVisualizationCallback(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  return publishVisualization();
}

bool BasePlannerRos::publishTrajectoryPointsCallback(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  return publishTrajectoryPoints();
}

mav_msgs::EigenOdometry BasePlannerRos::globalOdometryFromOdometry(
    const mav_msgs::EigenOdometry& odometry) const {
  // Check if odometry is already in local frame.
  if (odometry_in_global_frame_) {
    return odometry;
  } else {
    ROS_INFO_STREAM(
        output_prefix_
        << "Transforming odometry message from local frame using T_G_L:\n"
        << T_G_L_);
    mav_msgs::EigenOdometry odometry_global;
    odometry_global.position_W = T_G_L_ * odometry.position_W;
    odometry_global.orientation_W_B =
        T_G_L_.getRotation().toImplementation() * odometry.orientation_W_B;
    return odometry_global;
  }
}

// Reset the planner when a new polygon is set.
bool BasePlannerRos::resetPlanner() {
  ROS_ERROR_STREAM(kOutputPrefix << "resetPlanner is not implemented.");
  return false;
}

}  // namespace mav_coverage_planning
