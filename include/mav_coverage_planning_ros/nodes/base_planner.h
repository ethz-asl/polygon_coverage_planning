#ifndef MAV_COVERAGE_PLANNING_ROS_BASE_PLANNER_H_
#define MAV_COVERAGE_PLANNING_ROS_BASE_PLANNER_H_

#include <memory>

#include <mav_planning_msgs/PlannerService.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

// Planning structures.
#include <mav_2d_coverage_planning/geometry/polygon.h>
#include <mav_3d_coverage_planning/polyhedron.h>
#include <mav_coverage_planning_comm/trajectory_cost_functions.h>
#include <mav_coverage_planning_comm/trajectory_definitions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_planning_msgs/PolygonService.h>
#include <mav_trajectory_generation/trajectory.h>

// Stuff to receive MAV odometry:
#include <geometry_msgs/TransformStamped.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <minkindr_conversions/kindr_msg.h>
#include <nav_msgs/Odometry.h>

namespace mav_coverage_planning {

// The default control parameters
constexpr bool kDefaultLatchTopic = true;
const std::string kDefaultLocalFrameID = "odom";

// Default publishing behaviour
constexpr bool kDefaultPublishPlanPointsOnPlanningComplete = false;
constexpr bool kDefaultPublishVisualizationOnPlanningComplete = true;

typedef kindr::minimal::QuatTransformation Transformation;

class BasePlanner {
 public:
  struct BaseSettings {
    BaseSettings();
    TrajectoryCostFunctionType trajectory_cost_function;
    Polyhedron_3 raw_polyhedron;
    double altitude;
    Polygon polygon;
    Polyhedron_3 clipped_polyhedron;
    bool latch_topics;
    std::string local_frame_id;
    std::string global_frame_id;
    bool publish_plan_on_planning_complete;
    bool publish_visualization_on_planning_complete;
    enum CostFunctionType {
      kDistance = 0,  // Minimize distance.
      kTime           // Minimize flight time.
    } cost_function_type = kTime;

    inline std::string getCostFunctionTypeName() {
      switch (cost_function_type) {
        case CostFunctionType::kDistance:
          return "Euclidean distance";
        case CostFunctionType::kTime:
          return "Time";
        default:
          return "Unknown!";
      }
    }

    inline bool checkCostFunctionTypeValid() {
      return (cost_function_type == CostFunctionType::kDistance) ||
             (cost_function_type == CostFunctionType::kTime);
    }
  };

  // Constructor
  BasePlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

 protected:
  // Call to the actual planner.
  virtual bool solvePlanner(const mav_msgs::EigenTrajectoryPoint& start,
                            const mav_msgs::EigenTrajectoryPoint& goal) {
    return true;
  }
  // Reset the planner when a new world is set.
  virtual bool resetPlanner() { return true; }

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  BaseSettings settings_;

  // The solution trajectory for a given start and goal.
  mav_msgs::EigenTrajectoryPointVector waypoints_;
  mav_trajectory_generation::Trajectory trajectory_;

 private:
  // Set a new polygon through a service call.
  bool setPolygonCallback(
      mav_planning_msgs::PolygonService::Request& request,
      mav_planning_msgs::PolygonService::Response& response);
  // Base initial interactions with ROS
  void getBaseParametersFromRos();
  void subscribeToBaseTopics();
  void advertiseBaseTopics();

  // Solves the planning problem from start to goal.
  bool planPathCallback(mav_planning_msgs::PlannerService::Request& request,
                        mav_planning_msgs::PlannerService::Response& response);
  // Solves the planning graph with start and goal at current odometry.
  bool planPathFromAndToOdometryCallback(
      mav_planning_msgs::PlannerService::Request& request,
      mav_planning_msgs::PlannerService::Response& response);
  // Solves the planning graph with start at current odometry and goal set from
  // request.
  bool planPathFromOdometryToGoalCallback(
      mav_planning_msgs::PlannerService::Request& request,
      mav_planning_msgs::PlannerService::Response& response);
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
  void solve(const mav_msgs::EigenTrajectoryPoint& start,
             const mav_msgs::EigenTrajectoryPoint& goal);

  // Visualization
  void publishVisualization();
  // Publishing the plan
  bool publishTrajectoryPoints();

  // Set from parameters.
  void setCostFunction();
  void setPolygon();
  void setPolyhedronFromGridmap();
  void clip();

  // Helper function to convert odometry to global odometry.
  mav_msgs::EigenOdometry globalOdometryFromOdometry(
      const mav_msgs::EigenOdometry& odometry) const;
  // Helper function to set planning request start pose from current odometry.
  bool planningRequestStartPoseFromOdometry(
      mav_planning_msgs::PlannerService::Request* req) const;

  // Publishers and Services
  ros::Publisher marker_pub_;
  ros::Publisher raw_polyhedron_pub_;
  ros::Publisher clipped_polyhedron_pub_;
  ros::Publisher waypoint_list_pub_;
  ros::ServiceServer plan_path_srv_;
  ros::ServiceServer plan_path_from_and_to_odometry_srv_;
  ros::ServiceServer plan_path_from_odometry_to_goal_srv_;
  ros::ServiceServer publish_visualization_srv_;
  ros::ServiceServer publish_plan_points_srv_;
  ros::ServiceServer publish_all_srv_;

  ros::Subscriber odometry_sub_;  // Receives the robot odometry.
  // Receives TF from local odometry frame to global planning frame.
  ros::Subscriber T_G_L_sub_;

  // Planner status
  bool planning_complete_;

  // Members for receiving odometry.
  bool odometry_set_;
  bool odometry_in_global_frame_;
  mav_msgs::EigenOdometry odometry_;
  Transformation T_G_L_;
};
}  // namespace mav_coverage_planning

#endif  // MAV_COVERAGE_PLANNING_ROS_BASE_PLANNER_H_
