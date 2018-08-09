#include "mav_coverage_planning/ros_interface.h"

#include <limits>
#include <algorithm>

#include <geometry_msgs/Point.h>
#include <glog/logging.h>
#include <ros/ros.h>

namespace mav_coverage_planning {

const std::string kPrefix = kOutputPrefix + "ros_interface]: ";

void eigenTrajectoryPointVectorFromPath(
    const StdVector2d& waypoints, double altitude,
    mav_msgs::EigenTrajectoryPointVector* traj_points) {
  CHECK_NOTNULL(traj_points);

  traj_points->clear();
  traj_points->resize(waypoints.size());
  for (size_t i = 0; i < waypoints.size(); i++) {
    (*traj_points)[i].position_W =
        Eigen::Vector3d(waypoints[i].x(), waypoints[i].y(), altitude);
  }
}

void poseArrayMsgFromEigenTrajectoryPointVector(
    const mav_msgs::EigenTrajectoryPointVector& trajectory_points,
    const std::string& frame_id,
    geometry_msgs::PoseArray* trajectory_points_pose_array) {
  CHECK_NOTNULL(trajectory_points_pose_array);

  // Header
  trajectory_points_pose_array->header.frame_id = frame_id;
  // Converting and populating the message with all points
  for (const mav_msgs::EigenTrajectoryPoint& trajectory_point :
       trajectory_points) {
    geometry_msgs::PoseStamped msg;
    mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(trajectory_point, &msg);
    trajectory_points_pose_array->poses.push_back(msg.pose);
  }
}

void poseArrayMsgFromPath(
    const StdVector2d& waypoints, double altitude, const std::string& frame_id,
    geometry_msgs::PoseArray* trajectory_points_pose_array) {
  CHECK_NOTNULL(trajectory_points_pose_array);

  mav_msgs::EigenTrajectoryPointVector eigen_traj;
  eigenTrajectoryPointVectorFromPath(waypoints, altitude, &eigen_traj);
  poseArrayMsgFromEigenTrajectoryPointVector(eigen_traj, frame_id,
                                             trajectory_points_pose_array);
}

void msgMultiDofJointTrajectoryFromPath(
    const StdVector2d& waypoints, double altitude,
    trajectory_msgs::MultiDOFJointTrajectory* msg) {
  CHECK_NOTNULL(msg);

  mav_msgs::EigenTrajectoryPointVector eigen_traj;
  eigenTrajectoryPointVectorFromPath(waypoints, altitude, &eigen_traj);
  mav_msgs::msgMultiDofJointTrajectoryFromEigen(eigen_traj, msg);
}

void createMarkers(const StdVector2d& vertices, double altitude,
                   const std::string& frame_id, const std::string& ns,
                   const mav_visualization::Color& points_color,
                   const mav_visualization::Color& lines_color,
                   visualization_msgs::Marker* points,
                   visualization_msgs::Marker* line_strip) {
  CHECK_NOTNULL(points);
  CHECK_NOTNULL(line_strip);
  points->points.clear();
  line_strip->points.clear();

  points->header.frame_id = line_strip->header.frame_id = frame_id;
  points->header.stamp = line_strip->header.stamp = ros::Time::now();
  points->ns = line_strip->ns = ns;
  points->action = line_strip->action = visualization_msgs::Marker::ADD;
  points->pose.orientation.w = line_strip->pose.orientation.w = 1.0;

  points->id = 0;
  line_strip->id = 1;

  points->type = visualization_msgs::Marker::POINTS;
  line_strip->type = visualization_msgs::Marker::LINE_STRIP;

  points->scale.x = 0.2;
  points->scale.y = 0.2;
  line_strip->scale.x = 0.1;

  points->color = points_color;
  line_strip->color = lines_color;

  for (size_t i = 0; i < vertices.size(); i++) {
    geometry_msgs::Point p;
    p.x = vertices[i].x();
    p.y = vertices[i].y();
    p.z = altitude;

    points->points.push_back(p);
    line_strip->points.push_back(p);
  }
}

void createPolygonMarkers(const Polygon& polygon, double altitude,
                          const std::string& frame_id, const std::string& ns,
                          const mav_visualization::Color& polygon_color,
                          const mav_visualization::Color& hole_color,
                          visualization_msgs::MarkerArray* array) {
  CHECK_NOTNULL(array);
  array->markers.clear();

  // Polygon markers.
  visualization_msgs::Marker hull_points, hull_vertices;
  // Close hull.
  StdVector2d hull = polygon.getVertices();
  hull.push_back(polygon.getVertices().front());
  createMarkers(hull, altitude, frame_id, ns + "hull", polygon_color,
                polygon_color, &hull_points, &hull_vertices);
  array->markers.push_back(hull_points);
  array->markers.push_back(hull_vertices);

  // Hole markers.
  for (size_t i = 0; i < polygon.getNumHoles(); ++i) {
    visualization_msgs::Marker hole_points, hole_vertices;
    StdVector2d hole = polygon.getHoles()[i].getVertices();
    hole.push_back(polygon.getHoles()[i].getVertices().front());
    createMarkers(hole, altitude, frame_id, ns + "hole_" + std::to_string(i),
                  hole_color, hole_color, &hole_points, &hole_vertices);
    array->markers.push_back(hole_points);
    array->markers.push_back(hole_vertices);
  }
}

void createMarkers(const StdVector2d& vertices, double altitude,
                   visualization_msgs::Marker* points,
                   visualization_msgs::Marker* line_strip) {
  CHECK_NOTNULL(points);
  CHECK_NOTNULL(line_strip);
  createMarkers(vertices, altitude, "world", "vertices_and_strip",
                mav_visualization::Color::Red(),
                mav_visualization::Color::Green(), points, line_strip);
}

void createStartAndEndPointMarkers(const Eigen::Vector2d& start,
                                   const Eigen::Vector2d& end, double altitude,
                                   const std::string& frame_id,
                                   const std::string& ns,
                                   visualization_msgs::Marker* start_point,
                                   visualization_msgs::Marker* end_point) {
  CHECK_NOTNULL(start_point);
  CHECK_NOTNULL(end_point);

  start_point->header.frame_id = end_point->header.frame_id = frame_id;
  start_point->header.stamp = end_point->header.stamp = ros::Time::now();
  start_point->ns = ns + "_start";
  end_point->ns = ns + "_end";
  start_point->action = end_point->action = visualization_msgs::Marker::ADD;

  start_point->pose.position.x = start.x();
  start_point->pose.position.y = start.y();
  start_point->pose.position.z = altitude;

  end_point->pose.position.x = end.x();
  end_point->pose.position.y = end.y();
  end_point->pose.position.z = altitude;

  start_point->pose.orientation.w = end_point->pose.orientation.w = 1.0;

  start_point->id = end_point->id = 0;

  start_point->type = end_point->type = visualization_msgs::Marker::SPHERE;

  start_point->scale.x = end_point->scale.x = 1.0;
  start_point->scale.y = end_point->scale.y = 1.0;
  start_point->scale.z = end_point->scale.z = 1.0;

  start_point->color = mav_visualization::Color::Green();
  start_point->color.a = 0.5;

  end_point->color = mav_visualization::Color::Red();
  end_point->color.a = 0.5;
}

void verticesFromPolygonMsg(const planning_msgs::Polygon2D& msg,
                            StdVector2d* vertices) {
  CHECK_NOTNULL(vertices);

  vertices->resize(msg.points.size());
  for (size_t i = 0; i < msg.points.size(); ++i) {
    (*vertices)[i].x() = msg.points[i].x;
    (*vertices)[i].y() = msg.points[i].y;
  }
}

bool polygonFromMsg(const planning_msgs::PolygonWithHolesStamped& msg,
                    Polygon* polygon, double* altitude, std::string* frame) {
  CHECK_NOTNULL(polygon);
  CHECK_NOTNULL(altitude);
  CHECK_NOTNULL(frame);

  *frame = msg.header.frame_id;
  *altitude = msg.altitude;

  StdVector2d hull;
  verticesFromPolygonMsg(msg.polygon.hull, &hull);

  std::vector<StdVector2d> holes(msg.polygon.holes.size());
  for (size_t i = 0; i < msg.polygon.holes.size(); ++i) {
    verticesFromPolygonMsg(msg.polygon.holes[i], &holes[i]);
  }

  *polygon = Polygon(hull, holes);
  if (!polygon->isValid()) {
    ROS_ERROR_STREAM(kPrefix << "Input polygon is not valid.");
    return false;
  } else if (!polygon->isSimple()) {
    ROS_ERROR_STREAM(kPrefix << "Input polygon is not simple.");
    return false;
  } else if (polygon->hasHoles() && !polygon->hasSimpleHoles()) {
    ROS_ERROR_STREAM(kPrefix << "Input polygon has non-simple holes.");
    return false;
  } else {
    return true;
  }
}

}  // namespace mav_coverage_planning
