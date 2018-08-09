#ifndef MAV_COVERAGE_PLANNING_BASE_PLANNING_ROS_H_
#define MAV_COVERAGE_PLANNING_BASE_PLANNING_ROS_H_

#include <memory>

#include <Eigen/Core>

#include <planning_msgs/PlannerService.h>
#include <planning_msgs/PolygonService.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

// Stuff to receive MAV odometry:
#include <geometry_msgs/TransformStamped.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <minkindr_conversions/kindr_msg.h>
#include <nav_msgs/Odometry.h>

#include "mav_coverage_planning/common.h"
#include "mav_coverage_planning/cost_function.h"
#include "mav_coverage_planning/polygon.h"

namespace mav_coverage_planning {

// The default cost function type
const CostFunction::Type kDefaultCostFunctionType =
    CostFunction::Type::kVelocityRampTime;
// The default maximum velocity
constexpr double kDefaultMaximumVelocity = 2.0;
// The default maximum acceleration
constexpr double kDefaultMaximumAcceleration = 4.0;

// The default control parameters
constexpr bool kDefaultLatchTopic = true;
const std::string kDefaultLocalFrameID = "odom";

const std::string kDefaultOutputPrefix = kOutputPrefix + "base_planner_ros]: ";

// Default publishing behaviour
constexpr bool kDefaultPublishPlanPointsOnPlanningComplete = false;
constexpr bool kDefaultPublishVisualizationOnPlanningComplete = true;

typedef kindr::minimal::QuatTransformation Transformation;

// A basic ros wrapper for planner in a 2D polynomial environment.
class BasePlannerRos {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Constructor
  BasePlannerRos(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  // Initial interactions with ROS
  void getParametersFromRos();
  void subscribeToTopics();
  void advertiseTopics();

 protected:
  // Set a new polygon through a service call.
  bool setPolygonCallback(planning_msgs::PolygonService::Request& request,
                          planning_msgs::PolygonService::Response& response);
  // Solves the planning problem from start to goal.
  bool planPathCallback(planning_msgs::PlannerService::Request& request,
                        planning_msgs::PlannerService::Response& response);
  // Solves the planning graph with start and goal at current odometry.
  bool planPathFromAndToOdometryCallback(
      planning_msgs::PlannerService::Request& request,
      planning_msgs::PlannerService::Response& response);
  // Solves the planning graph with start at current odometry and goal set from
  // request.
  bool planPathFromOdometryToGoalCallback(
      planning_msgs::PlannerService::Request& request,
      planning_msgs::PlannerService::Response& response);
  bool publishAllCallback(std_srvs::Empty::Request& request,
                          std_srvs::Empty::Response& response);
  bool publishVisualizationCallback(std_srvs::Empty::Request& request,
                                    std_srvs::Empty::Response& response);
  bool publishTrajectoryPointsCallback(std_srvs::Empty::Request& request,
                                       std_srvs::Empty::Response& response);

  void receiveOdometryCallback(const nav_msgs::Odometry& msg);
  void receiveTransformCallback(const geometry_msgs::TransformStamped& msg);

  // Solve the planning problem. Stores status planning_complete_ and publishes
  // trajectory and visualization if enabled.
  void solve(const Eigen::Vector2d& start, const Eigen::Vector2d& goal);
  // Call to the actual planner.
  // Note: Needs to be implemented!
  virtual bool solvePlanner(const Eigen::Vector2d& start,
                            const Eigen::Vector2d& goal);
  // Reset the planner when a new polygon is set.
  // Note: Needs to be implemented!
  virtual bool resetPlanner();

  // Visualization
  bool publishVisualization();
  // Publishing the plan
  bool publishTrajectoryPoints();

  // Helper function to convert odometry to global odometry.
  mav_msgs::EigenOdometry globalOdometryFromOdometry(
      const mav_msgs::EigenOdometry& odometry) const;
  // Helper function to set planning request start pose from current odometry.
  bool planningRequestStartPoseFromOdometry(
      planning_msgs::PlannerService::Request* req) const;

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Publishers and Services
  ros::Publisher marker_pub_;
  ros::Publisher waypoint_list_pub_;
  ros::ServiceServer set_polygon_srv_;
  ros::ServiceServer plan_path_srv_;
  ros::ServiceServer plan_path_from_and_to_odometry_srv_;
  ros::ServiceServer plan_path_from_odometry_to_goal_srv_;
  ros::ServiceServer publish_visualization_srv_;
  ros::ServiceServer publish_plan_points_srv_;
  ros::ServiceServer publish_all_srv_;

  ros::Subscriber odometry_sub_;  // Receives the robot odometry.
  // Receives TF from local odometry frame to global planning frame.
  ros::Subscriber T_G_L_sub_;

  // The solution waypoints for a given start and goal.
  StdVector2d solution_;
  // Planner status
  bool planning_complete_;

  // Members for receiving odometry.
  bool odometry_set_;
  bool odometry_in_global_frame_;
  mav_msgs::EigenOdometry odometry_;
  Transformation T_G_L_;
  std::string global_frame_id_;
  std::string local_frame_id_;

  // System Parameters
  double altitude_;

  Polygon polygon_;
  CostFunction cost_function_;

  // Control Parameters
  bool latch_topic_;
  std::string output_prefix_;

  // Flags
  bool publish_plan_points_on_planning_complete_;
  bool publish_visualization_on_planning_complete_;
};
}  // namespace mav_coverage_planning

#endif  // MAV_COVERAGE_PLANNING_BASE_PLANNING_ROS_H_
