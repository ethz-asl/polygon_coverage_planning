#include <Eigen/Core>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <mav_visualization/helpers.h>

#include "mav_coverage_planning/common.h"
#include "mav_coverage_planning/ros_interface.h"
#include "mav_coverage_planning/sweep_planner.h"

using namespace mav_coverage_planning;

const double kAltitude = 2.0;             // The constant altitude of the MAV.
const double kFOVCameraRad = M_PI / 2.0;  // The FOV of the camera.
const double kMinViewOverlap =
    0.2;  // The minimum view overlap between two sweeps [0 .. 1)

const StdVector2d kPolygonVertices = {
    Eigen::Vector2d(-30.0, 30.0), Eigen::Vector2d(30.0, 30.0),
    Eigen::Vector2d(30.0, 10.0),  Eigen::Vector2d(60.0, 0.0),
    Eigen::Vector2d(0.0, -40.0),  Eigen::Vector2d(-40.0, 0.0)};  // Hull

const StdVector2d kHole = {
    Eigen::Vector2d(0.0, 15.0),   Eigen::Vector2d(0.0, 10.0),
    Eigen::Vector2d(-5.0, 10.0),  Eigen::Vector2d(-5.0, 15.0),
    Eigen::Vector2d(-15.0, 15.0), Eigen::Vector2d(-15.0, 0.0),
    Eigen::Vector2d(10.0, 0.0),   Eigen::Vector2d(10.0, 15.0)};  // Hole.

const std::vector<StdVector2d> kHoles = {kHole};

const Eigen::Vector2d kStart(0.0, 30.0);
const Eigen::Vector2d kGoal(0.0, -40.0);

int main(int argc, char** argv) {
  // ROS stuff.
  ros::init(argc, argv, "coverage_plan_viz");
  ros::NodeHandle n;
  ros::Publisher marker_pub =
      n.advertise<visualization_msgs::MarkerArray>("plan_markers", 1, true);
  visualization_msgs::MarkerArray markers;
  ros::Rate loop_rate(10.0);

  // Coverage planning.
  Polygon p(kPolygonVertices, kHoles);
  CostFunction cost_function;  // Euclidean cost.
  SweepPlanner sp(p, cost_function, kAltitude, kFOVCameraRad, kMinViewOverlap,
                  GTSPPSolver::kExactWithPreprocessing);
  StdVector2d solution;
  if (!sp.solve(kStart, kGoal, &solution)) {
    ROS_ERROR_STREAM("Cannot create coverage plan for polygon " << p);
    ros::shutdown();
  }

  // (Optional) Convert to EigenTrajectoryPointVector.
  mav_msgs::EigenTrajectoryPointVector trajectory_points;
  eigenTrajectoryPointVectorFromPath(solution, kAltitude, &trajectory_points);

  // RVIZ visualization.
  // The planned path:
  visualization_msgs::Marker path_points, path_line_strips;
  createMarkers(solution, kAltitude, &path_points, &path_line_strips);
  markers.markers.push_back(path_points);
  markers.markers.push_back(path_line_strips);

  // The original polygon:
  visualization_msgs::MarkerArray polygon;
  createPolygonMarkers(p, kAltitude, "world", "polygon",
                       mav_visualization::Color::Blue(),
                       mav_visualization::Color::Red(), &polygon);
  markers.markers.insert(markers.markers.end(), polygon.markers.begin(),
                         polygon.markers.end());

  marker_pub.publish(markers);
  while (ros::ok()) {
    ros::spinOnce();
  }
}
