#include <cstdlib>
#include <string>

#include <Eigen/Core>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <mav_visualization/helpers.h>

#include "mav_coverage_planning/common.h"
#include "mav_coverage_planning/cost_function.h"
#include "mav_coverage_planning/math.h"
#include "mav_coverage_planning/polygon.h"
#include "mav_coverage_planning/ros_interface.h"
#include "mav_coverage_planning/sweep_planner.h"

double createRandomDouble(double min, double max) {
  // No seed for repeatability.
  return (max - min) * (static_cast<double>(std::rand()) /
                        static_cast<double>(RAND_MAX)) +
         min;
}

Eigen::Vector2d createRandom2DVector(double min, double max) {
  // No seed for repeatability.
  return Eigen::Vector2d(createRandomDouble(min, max),
                         createRandomDouble(min, max));
}

using namespace mav_coverage_planning;

const double kPolygonDiameter = 15.0;
const double kAltitude = 2.0;
const double kFOVCameraRad = M_PI / 2.0;
const double kMinViewOverlap = 0.2;
const double kDistancePolygonCenters = kPolygonDiameter + 2.0;
const int kSqrtNumPolygons = 10;

int main(int argc, char** argv) {
  // ROS stuff.
  ros::init(argc, argv, "coverage_plan_viz");
  ros::NodeHandle n;
  ros::Publisher marker_pub =
      n.advertise<visualization_msgs::MarkerArray>("plan_markers", 1, true);
  ros::Rate loop_rate(10.0);
  visualization_msgs::MarkerArray markers;

  // Coverage planning.
  // Create random polygons.
  std::srand(123456);
  StdVector2d cw_polygon_vertices;
  double x_0, y_0;
  x_0 = y_0 = 0.0;
  // Sample start and goal around polygon bounding box.
  Eigen::Vector2d start =
      createRandom2DVector(-kPolygonDiameter / 2.0, kPolygonDiameter / 2.0);
  Eigen::Vector2d goal =
      createRandom2DVector(-kPolygonDiameter / 2.0, kPolygonDiameter / 2.0);
  int num_polygons = kSqrtNumPolygons * kSqrtNumPolygons;
  for (int i = 0; i < num_polygons; i++) {
    // Shift y center, reset x center.
    if ((i % kSqrtNumPolygons) == 0 && i != 0) {
      y_0 += kDistancePolygonCenters;
      start.y() += kDistancePolygonCenters;
      goal.y() += kDistancePolygonCenters;
      x_0 = 0.0;
      start.x() = 0.0;
      goal.x() = 0.0;
    }

    // Create random polygons and paths.
    createRandomConvexPolygon(x_0, y_0, kPolygonDiameter / 2.0,
                              &cw_polygon_vertices);
    Polygon p(cw_polygon_vertices);
    CostFunction cost_function;  // Euclidean cost.
    SweepPlanner sp(p, cost_function, kAltitude, kFOVCameraRad, kMinViewOverlap,
                    GTSPPSolver::kExactWithPreprocessing);

    StdVector2d solution;
    if (!sp.solve(start, goal, &solution)) {
      ROS_ERROR_STREAM("Cannot create coverage plan for polygon " << p);
      continue;
    }

    // Shift x center.
    x_0 += kDistancePolygonCenters;
    start.x() += kDistancePolygonCenters;
    goal.x() += kDistancePolygonCenters;

    // RVIZ visualization.
    // The planned path:
    visualization_msgs::Marker path_points, path_line_strips;
    createMarkers(solution, kAltitude, "world", "path_" + std::to_string(i),
                  mav_visualization::Color::Red(),
                  mav_visualization::Color::Green(), &path_points,
                  &path_line_strips);
    markers.markers.push_back(path_points);
    markers.markers.push_back(path_line_strips);

    // The original polygon:
    visualization_msgs::Marker polygon_vertices, polygon_edges;
    StdVector2d cw_polygon_vertices_closed = p.getVertices();
    cw_polygon_vertices_closed.push_back(p.getVertices().front());
    createMarkers(
        cw_polygon_vertices_closed, kAltitude, "world",
        "polygon_" + std::to_string(i), mav_visualization::Color::Blue(),
        mav_visualization::Color::Blue(), &polygon_vertices, &polygon_edges);
    markers.markers.push_back(polygon_vertices);
    markers.markers.push_back(polygon_edges);
  }

  marker_pub.publish(markers);
  while (ros::ok()) {
    ros::spinOnce();
  }
}
