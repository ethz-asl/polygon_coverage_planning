/*
 * polygon_coverage_planning implements algorithms for coverage planning in
 * general polygons with holes. Copyright (C) 2019, Rik Bähnemann, Autonomous
 * Systems Lab, ETH Zürich
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "polygon_coverage_ros/ros_interface.h"

#include <algorithm>
#include <limits>

#include <geometry_msgs/Point.h>
#include <polygon_coverage_geometry/boolean.h>
#include <polygon_coverage_geometry/cgal_comm.h>
#include <polygon_coverage_geometry/cgal_definitions.h>
#include <polygon_coverage_geometry/triangulation.h>
#include <ros/assert.h>
#include <ros/ros.h>
#include <Eigen/Core>

namespace polygon_coverage_planning {

void poseArrayMsgFromPath(const std::vector<Point_2>& waypoints,
                          double altitude, const std::string& frame_id,
                          geometry_msgs::PoseArray* pose_array) {
  ROS_ASSERT(pose_array);

  pose_array->header.frame_id = frame_id;
  pose_array->poses.clear();
  pose_array->poses.resize(waypoints.size());
  for (size_t i = 0; i < waypoints.size(); i++) {
    msgPointFromWaypoint(waypoints[i], altitude,
                         &pose_array->poses[i].position);
  }
}

void msgMultiDofJointTrajectoryFromPath(
    const std::vector<Point_2>& waypoints, double altitude,
    trajectory_msgs::MultiDOFJointTrajectory* msg) {
  ROS_ASSERT(msg);

  geometry_msgs::PoseArray pose_array;
  poseArrayMsgFromPath(waypoints, altitude, "", &pose_array);

  msg->header = pose_array.header;
  msg->joint_names.clear();
  msg->joint_names.push_back("base_link");
  msg->points.clear();

  for (const auto& pose : pose_array.poses) {
    trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg;
    point_msg.transforms.resize(1);
    point_msg.velocities.resize(1);
    point_msg.accelerations.resize(1);

    point_msg.transforms[0].translation.x = pose.position.x;
    point_msg.transforms[0].translation.y = pose.position.y;
    point_msg.transforms[0].translation.z = pose.position.z;

    point_msg.transforms[0].rotation.w = 1.0;

    msg->points.push_back(point_msg);
  }
}

void msgPointFromWaypoint(const Point_2& waypoint, double altitude,
                          geometry_msgs::Point* point) {
  ROS_ASSERT(point);

  point->x = CGAL::to_double(waypoint.x());
  point->y = CGAL::to_double(waypoint.y());
  point->z = altitude;
}

void createMarkers(const std::vector<Point_2>& vertices, double altitude,
                   const std::string& frame_id, const std::string& ns,
                   const Color& points_color, const Color& lines_color,
                   const double line_size, const double point_size,
                   visualization_msgs::Marker* points,
                   visualization_msgs::Marker* line_strip) {
  ROS_ASSERT(points);
  ROS_ASSERT(line_strip);
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

  points->scale.x = point_size;
  points->scale.y = point_size;
  line_strip->scale.x = line_size;

  points->color = points_color;
  line_strip->color = lines_color;

  for (size_t i = 0; i < vertices.size(); i++) {
    geometry_msgs::Point p;
    msgPointFromWaypoint(vertices[i], altitude, &p);

    points->points.push_back(p);
    line_strip->points.push_back(p);
  }
}

void createPolygonMarkers(const PolygonWithHoles& polygon, double altitude,
                          const std::string& frame_id, const std::string& ns,
                          const Color& polygon_color, const Color& hole_color,
                          const double line_size, const double point_size,
                          visualization_msgs::MarkerArray* array) {
  ROS_ASSERT(array);
  array->markers.clear();

  // Polygon markers.
  visualization_msgs::Marker hull_points, hull_vertices;
  // Hull.
  std::vector<Point_2> hull = getHullVertices(polygon);
  hull.push_back(hull.front());
  createMarkers(hull, altitude, frame_id, ns + "hull", polygon_color,
                polygon_color, line_size, point_size, &hull_points,
                &hull_vertices);
  array->markers.push_back(hull_points);
  array->markers.push_back(hull_vertices);

  // Hole markers:
  size_t i = 0;
  for (auto h = polygon.holes_begin(); h != polygon.holes_end(); ++h) {
    visualization_msgs::Marker hole_tris;
    // Faces.
    std::vector<std::vector<Point_2>> triangles;
    triangulatePolygon(PolygonWithHoles(*h), &triangles);
    createTriangles(triangles, frame_id,
                    ns + "hole_mesh_" + std::to_string(i++), hole_color,
                    altitude, &hole_tris);
    array->markers.push_back(hole_tris);
  }
}

void createStartAndEndPointMarkers(const Point_2& start, const Point_2& end,
                                   double altitude, const std::string& frame_id,
                                   const std::string& ns,
                                   visualization_msgs::Marker* start_point,
                                   visualization_msgs::Marker* end_point) {
  ROS_ASSERT(start_point);
  ROS_ASSERT(end_point);

  geometry_msgs::Pose start_pose;
  msgPointFromWaypoint(start, altitude, &start_pose.position);

  geometry_msgs::Pose end_pose;
  msgPointFromWaypoint(end, altitude, &end_pose.position);

  return createStartAndEndPointMarkers(start_pose, end_pose, frame_id, ns,
                                       start_point, end_point);
}

void createStartAndEndPointMarkers(const geometry_msgs::Pose& start,
                                   const geometry_msgs::Pose& end,
                                   const std::string& frame_id,
                                   const std::string& ns,
                                   visualization_msgs::Marker* start_point,
                                   visualization_msgs::Marker* end_point) {
  ROS_ASSERT(start_point);
  ROS_ASSERT(end_point);

  start_point->header.frame_id = end_point->header.frame_id = frame_id;
  start_point->header.stamp = end_point->header.stamp = ros::Time::now();
  start_point->ns = ns + "_start";
  end_point->ns = ns + "_end";
  start_point->action = end_point->action = visualization_msgs::Marker::ADD;

  start_point->pose = start;
  end_point->pose = end;

  start_point->pose.orientation.w = end_point->pose.orientation.w = 1.0;

  start_point->id = end_point->id = 0;

  start_point->type = end_point->type = visualization_msgs::Marker::SPHERE;

  start_point->scale.x = end_point->scale.x = 1.0;
  start_point->scale.y = end_point->scale.y = 1.0;
  start_point->scale.z = end_point->scale.z = 1.0;

  start_point->color = Color::Green();
  start_point->color.a = 0.5;

  end_point->color = Color::Red();
  end_point->color.a = 0.5;
}

void createStartAndEndTextMarkers(const Point_2& start, const Point_2& end,
                                  double altitude, const std::string& frame_id,
                                  const std::string& ns,
                                  visualization_msgs::Marker* start_text,
                                  visualization_msgs::Marker* end_text) {
  ROS_ASSERT(start_text);
  ROS_ASSERT(end_text);

  geometry_msgs::Pose start_pose;
  msgPointFromWaypoint(start, altitude, &start_pose.position);

  geometry_msgs::Pose end_pose;
  msgPointFromWaypoint(end, altitude, &end_pose.position);

  return createStartAndEndTextMarkers(start_pose, end_pose, frame_id, ns,
                                      start_text, end_text);
}

void createStartAndEndTextMarkers(const geometry_msgs::Pose& start,
                                  const geometry_msgs::Pose& end,
                                  const std::string& frame_id,
                                  const std::string& ns,
                                  visualization_msgs::Marker* start_text,
                                  visualization_msgs::Marker* end_text) {
  ROS_ASSERT(start_text);
  ROS_ASSERT(end_text);

  start_text->header.frame_id = end_text->header.frame_id = frame_id;
  start_text->header.stamp = end_text->header.stamp = ros::Time::now();
  start_text->ns = ns + "_start_text";
  start_text->ns = ns + "_end_text";
  start_text->action = end_text->action = visualization_msgs::Marker::ADD;
  start_text->type = end_text->type =
      visualization_msgs::Marker::TEXT_VIEW_FACING;

  start_text->pose = start;
  end_text->pose = end;

  start_text->pose.orientation.w = end_text->pose.orientation.w = 1.0;

  start_text->text = "S";
  start_text->color = Color::Black();

  end_text->text = "G";
  end_text->color = Color::Black();

  start_text->scale.z = end_text->scale.z = 1.0;
}

void polygon2FromPolygonMsg(const geometry_msgs::Polygon& msg,
                            Polygon_2* polygon) {
  ROS_ASSERT(polygon);

  std::vector<Point_2> vertices(msg.points.size());
  for (size_t i = 0; i < msg.points.size(); ++i)
    vertices[i] = Point_2(msg.points[i].x, msg.points[i].y);

  *polygon = Polygon_2(vertices.begin(), vertices.end());
}

bool polygonFromMsg(const polygon_coverage_msgs::PolygonWithHolesStamped& msg,
                    PolygonWithHoles* polygon, double* altitude,
                    std::string* frame) {
  ROS_ASSERT(polygon);
  ROS_ASSERT(altitude);
  ROS_ASSERT(frame);

  *frame = msg.header.frame_id;

  if (msg.polygon.hull.points.size() > 0) {
    *altitude = msg.polygon.hull.points[0].z;
    ROS_INFO_STREAM(
        "Setting polygon altitude height to first z variable: " << *altitude);
  } else {
    ROS_ERROR("Polygon hull data empty. Cannot set altitude.");
    return false;
  }

  Polygon_2 hull;
  polygon2FromPolygonMsg(msg.polygon.hull, &hull);
  if (hull.is_clockwise_oriented()) hull.reverse_orientation();

  std::list<Polygon_2> holes(msg.polygon.holes.size());
  for (size_t i = 0; i < msg.polygon.holes.size(); ++i) {
    auto hole = std::next(holes.begin(), i);
    polygon2FromPolygonMsg(msg.polygon.holes[i], &(*hole));
    if (hole->is_counterclockwise_oriented()) hole->reverse_orientation();
  }

  // Cut holes from hull.
  std::list<PolygonWithHoles> res =
      polygon_coverage_planning::computeDifference(hull, holes.begin(),
                                                   holes.end());
  if (res.empty()) {
    ROS_ERROR("Failed to create polygon with holes from msg.");
    return false;
  }
  *polygon = res.front();

  if (polygon->outer_boundary().size() < 3) {
    ROS_ERROR_STREAM("Input polygon is not valid.");
    return false;
  } else if (!isStrictlySimple(*polygon)) {
    ROS_ERROR_STREAM("Input polygon is not simple.");
    return false;
  } else if (res.size() > 1) {
    ROS_ERROR(
        "Failed to create polygon from message. More than two input polygons "
        "after cropping holes from hull.");
    return false;
  }

  return true;
}
void createTriangles(const std::vector<std::vector<Point_2>>& triangles,
                     const std::string& frame_id, const std::string& ns,
                     const Color& color, const double altitude,
                     visualization_msgs::Marker* markers) {
  ROS_ASSERT(markers);

  markers->points.clear();

  markers->header.frame_id = frame_id;
  markers->header.stamp = ros::Time::now();
  markers->ns = ns;
  markers->action = visualization_msgs::Marker::ADD;
  markers->pose.orientation.w = 1.0;
  markers->scale.x = 1.0;
  markers->scale.y = 1.0;
  markers->scale.z = 1.0;

  markers->id = 0;
  markers->type = visualization_msgs::Marker::TRIANGLE_LIST;

  markers->color = color;
  for (const auto& t : triangles) {
    ROS_ASSERT(t.size() == 3);
    for (const auto& v : t) {
      geometry_msgs::Point p;
      msgPointFromWaypoint(v, altitude, &p);

      markers->points.push_back(p);
    }
  }
}

}  // namespace polygon_coverage_planning
