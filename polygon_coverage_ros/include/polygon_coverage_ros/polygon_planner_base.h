#ifndef POLYGON_COVERAGE_ROS_POLYGON_PLANNER_BASE_H_
#define POLYGON_COVERAGE_ROS_POLYGON_PLANNER_BASE_H_

#include <memory>
#include <optional>

#include <polygon_coverage_geometry/cgal_definitions.h>
#include <polygon_coverage_msgs/PolygonService.h>
#include <polygon_coverage_planners/cost_functions/path_cost_functions.h>
#include <polygon_coverage_planners/sensor_models/sensor_model_base.h>

#include <mav_planning_msgs/PlannerService.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/MarkerArray.h>

namespace polygon_coverage_planning {

// A basic ros wrapper for planner in a 2D polynomial environment.
class PolygonPlannerBase {
 public:
  // Constructor
  PolygonPlannerBase(const ros::NodeHandle& nh,
                     const ros::NodeHandle& nh_private);

 private:
  // Initial interactions with ROS
  void getParametersFromRos();
  void advertiseTopics();

  // Call to the actual planner.
  bool solvePlanner(const Point_2& start, const Point_2& goal);
  // Reset the planner when a new polygon is set.
  bool resetPlanner();

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // The solution waypoints for a given start and goal.
  std::vector<Point_2> solution_;

  // Set a new polygon through a service call.
  bool setPolygonCallback(
      polygon_coverage_msgs::PolygonService::Request& request,
      polygon_coverage_msgs::PolygonService::Response& response);
  // Solves the planning problem from start to goal.
  bool planPathCallback(mav_planning_msgs::PlannerService::Request& request,
                        mav_planning_msgs::PlannerService::Response& response);
  bool publishAllCallback(std_srvs::Empty::Request& request,
                          std_srvs::Empty::Response& response);
  bool publishVisualizationCallback(std_srvs::Empty::Request& request,
                                    std_srvs::Empty::Response& response);
  bool publishTrajectoryPointsCallback(std_srvs::Empty::Request& request,
                                       std_srvs::Empty::Response& response);

  // Solve the planning problem. Stores status planning_complete_ and publishes
  // trajectory and visualization if enabled.
  void solve(const Point_2& start, const Point_2& goal);

  // Visualization
  bool publishVisualization();
  visualization_msgs::MarkerArray createDecompositionMarkers();

  // Publishing the plan
  bool publishTrajectoryPoints();
  // Publishers and Services
  ros::Publisher marker_pub_;
  ros::Publisher waypoint_list_pub_;
  ros::ServiceServer set_polygon_srv_;
  ros::ServiceServer plan_path_srv_;
  ros::ServiceServer publish_visualization_srv_;
  ros::ServiceServer publish_plan_points_srv_;
  ros::ServiceServer publish_all_srv_;

  // Planner status
  bool planning_complete_;

  visualization_msgs::MarkerArray markers_;

  // Parameters
  std::optional<PolygonWithHoles> polygon_;
  double wall_distance_;
  std::pair<PathCostFunction, CostFunctionType> path_cost_function_;
  std::optional<double> altitude_;
  bool latch_topics_;
  std::string global_frame_id_;
  bool publish_plan_on_planning_complete_;
  bool publish_visualization_on_planning_complete_;
  std::optional<double> v_max_;
  std::optional<double> a_max_;
};
}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_ROS_POLYGON_PLANNER_BASE_H_
