#include "polygon_coverage_ros/shortest_path_planner.h"

#include <geometry_msgs/PointStamped.h>
#include <polygon_coverage_geometry/cgal_definitions.h>
#include <polygon_coverage_geometry/offset.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <ros/topic.h>

namespace polygon_coverage_planning {

ShortestPathPlanner::ShortestPathPlanner(const ros::NodeHandle& nh,
                                         const ros::NodeHandle& nh_private)
    : PolygonPlannerBase(nh, nh_private) {
  // Creating the visibility graph from the received parameters.
  // This operation may take some time.
  resetPlanner();

  // Advertise services.
  clicked_point_srv_ = nh_private_.advertiseService(
      "plan_from_clicked_points", &ShortestPathPlanner::setFromClickedPoint,
      this);
}

bool ShortestPathPlanner::solvePlanner(const Point_2& start,
                                       const Point_2& goal) {
  return planner_->solveWithOutsideStartAndGoal(start, goal, &solution_);
}

// Reset the planner when a new polygon is set.
bool ShortestPathPlanner::resetPlanner() {
  ROS_INFO_STREAM("Start creating the shortest plan graph.");
  if (!polygon_.has_value()) {
    ROS_WARN("No polygon set. Cannot reset planner.");
    return false;
  }
  PolygonWithHoles temp_poly = polygon_.value();
  computeOffsetPolygon(temp_poly, wall_distance_, &polygon_.value());
  planner_.reset(new visibility_graph::VisibilityGraph(polygon_.value()));
  if (planner_->isInitialized()) {
    ROS_INFO("Finished creating the shortest plan graph.");
    return true;
  } else {
    ROS_ERROR("Failed creating shortest path planner from user input.");
    return false;
  }
}

bool ShortestPathPlanner::setFromClickedPoint(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  const double kTimeOutS = 30.0;

  ROS_INFO("Select START using RVIZ PublishPoint tool.");
  geometry_msgs::PointStampedConstPtr start_msg =
      ros::topic::waitForMessage<geometry_msgs::PointStamped>(
          "/clicked_point", nh_, ros::Duration(kTimeOutS));
  if (start_msg == nullptr) {
    ROS_WARN("Timout start selection.");
    return false;
  }
  Point_2 start(start_msg->point.x, start_msg->point.y);

  ROS_INFO("Select GOAL using RVIZ PublishPoint tool.");
  geometry_msgs::PointStampedConstPtr goal_msg =
      ros::topic::waitForMessage<geometry_msgs::PointStamped>(
          "/clicked_point", nh_, ros::Duration(kTimeOutS));
  if (goal_msg == nullptr) {
    ROS_WARN("Timout goal selection.");
    return false;
  }
  Point_2 goal(goal_msg->point.x, goal_msg->point.y);

  solve(start, goal);
  return true;
}

}  // namespace polygon_coverage_planning
