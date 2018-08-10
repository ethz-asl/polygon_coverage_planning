#ifndef MAV_COVERAGE_PLANNING_BASE_PLANNER_H_
#define MAV_COVERAGE_PLANNING_BASE_PLANNER_H_

#include <memory>

#include <mav_2d_coverage_planning/definitions.h>
#include <mav_2d_coverage_planning/polygon.h>

#include <planning_msgs/PlannerService.h>
#include <planning_msgs/PolygonService.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

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

// A basic ros wrapper for planner in a 2D polynomial environment.
class BasePlanner2D {
 public:
  struct Settings {
    Settings();
    Polygon polygon;
    SegmentCostFunctionType visibility_graph_cost_function;
    PathCostFunctionType sweep_cost_function;
    double altitude;
    bool latch_topics;
    std::string local_frame_id;
    std::string global_frame_id;
    bool publish_plan_on_planning_complete;
    bool publish_visualization_on_planning_complete;
    enum CostFunctionType {
      kDistance = 0,  // Minimize distance.
      kTime           // Minimize flight time.
    } cost_function_type = kDistance;

    inline std::string getCostFunctionTypeName(
        ) {
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
  BasePlanner2D(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  // Initial interactions with ROS
  void getParametersFromRos();
  void subscribeToTopics();
  void advertiseTopics();

 protected:
  // Call to the actual planner.
  virtual bool solvePlanner(const Point_2& start, const Point_2& goal) = 0;
  // Reset the planner when a new polygon is set.
  virtual bool resetPlanner() = 0;

 private:
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
  void solve(const Point_2& start, const Point_2& goal);

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
  std::vector<Point_2> solution_;
  // Planner status
  bool planning_complete_;

  // Members for receiving odometry.
  bool odometry_set_;
  bool odometry_in_global_frame_;
  mav_msgs::EigenOdometry odometry_;
  Transformation T_G_L_;

  Settings settings_;
};
}  // namespace mav_coverage_planning

#endif  // MAV_COVERAGE_PLANNING_BASE_PLANNER_H_
