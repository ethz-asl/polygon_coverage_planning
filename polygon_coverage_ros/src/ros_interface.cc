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
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <polygon_coverage_geometry/cgal_comm.h>
#include <polygon_coverage_geometry/cgal_definitions.h>
#include <polygon_coverage_geometry/triangulation.h>
#include <ros/assert.h>
#include <ros/ros.h>
#include <Eigen/Core>

namespace polygon_coverage_planning {

void eigenTrajectoryPointVectorFromPath(
    const std::vector<Point_2>& waypoints, double altitude,
    mav_msgs::EigenTrajectoryPointVector* traj_points) {
  ROS_ASSERT(traj_points);

  traj_points->clear();
  traj_points->resize(waypoints.size());
  for (size_t i = 0; i < waypoints.size(); i++) {
    (*traj_points)[i].position_W =
        Eigen::Vector3d(CGAL::to_double(waypoints[i].x()),
                        CGAL::to_double(waypoints[i].y()), altitude);
  }
}

void poseArrayMsgFromEigenTrajectoryPointVector(
    const mav_msgs::EigenTrajectoryPointVector& trajectory_points,
    const std::string& frame_id,
    geometry_msgs::PoseArray* trajectory_points_pose_array) {
  ROS_ASSERT(trajectory_points_pose_array);

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
    const std::vector<Point_2>& waypoints, double altitude,
    const std::string& frame_id,
    geometry_msgs::PoseArray* trajectory_points_pose_array) {
  ROS_ASSERT(trajectory_points_pose_array);

  mav_msgs::EigenTrajectoryPointVector eigen_traj;
  eigenTrajectoryPointVectorFromPath(waypoints, altitude, &eigen_traj);
  poseArrayMsgFromEigenTrajectoryPointVector(eigen_traj, frame_id,
                                             trajectory_points_pose_array);
}

void msgMultiDofJointTrajectoryFromPath(
    const std::vector<Point_2>& waypoints, double altitude,
    trajectory_msgs::MultiDOFJointTrajectory* msg) {
  ROS_ASSERT(msg);

  mav_msgs::EigenTrajectoryPointVector eigen_traj;
  eigenTrajectoryPointVectorFromPath(waypoints, altitude, &eigen_traj);
  mav_msgs::msgMultiDofJointTrajectoryFromEigen(eigen_traj, msg);
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
    p.x = CGAL::to_double(vertices[i].x());
    p.y = CGAL::to_double(vertices[i].y());
    p.z = altitude;

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
  mav_msgs::EigenTrajectoryPoint eigen_start;
  eigen_start.position_W.x() = CGAL::to_double(start.x());
  eigen_start.position_W.y() = CGAL::to_double(start.y());
  eigen_start.position_W.z() = altitude;

  mav_msgs::EigenTrajectoryPoint eigen_end;
  eigen_end.position_W.x() = CGAL::to_double(end.x());
  eigen_end.position_W.y() = CGAL::to_double(end.y());
  eigen_end.position_W.z() = altitude;

  return createStartAndEndPointMarkers(eigen_start, eigen_end, frame_id, ns,
                                       start_point, end_point);
}

void createStartAndEndPointMarkers(const mav_msgs::EigenTrajectoryPoint& start,
                                   const mav_msgs::EigenTrajectoryPoint& end,
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

  geometry_msgs::PoseStamped start_stamped, end_stamped;
  msgPoseStampedFromEigenTrajectoryPoint(start, &start_stamped);
  msgPoseStampedFromEigenTrajectoryPoint(end, &end_stamped);

  start_point->pose = start_stamped.pose;
  end_point->pose = end_stamped.pose;

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
  mav_msgs::EigenTrajectoryPoint eigen_start;
  eigen_start.position_W.x() = CGAL::to_double(start.x());
  eigen_start.position_W.y() = CGAL::to_double(start.y());
  eigen_start.position_W.z() = altitude;

  mav_msgs::EigenTrajectoryPoint eigen_end;
  eigen_end.position_W.x() = CGAL::to_double(end.x());
  eigen_end.position_W.y() = CGAL::to_double(end.y());
  eigen_end.position_W.z() = altitude;

  return createStartAndEndTextMarkers(eigen_start, eigen_end, frame_id, ns,
                                      start_text, end_text);
}

void createStartAndEndTextMarkers(const mav_msgs::EigenTrajectoryPoint& start,
                                  const mav_msgs::EigenTrajectoryPoint& end,
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

  geometry_msgs::PoseStamped start_stamped, end_stamped;
  msgPoseStampedFromEigenTrajectoryPoint(start, &start_stamped);
  msgPoseStampedFromEigenTrajectoryPoint(end, &end_stamped);

  start_text->pose = start_stamped.pose;
  end_text->pose = end_stamped.pose;

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
  *polygon = PolygonWithHoles(hull);

  for (size_t i = 0; i < msg.polygon.holes.size(); ++i) {
    Polygon_2 hole;
    polygon2FromPolygonMsg(msg.polygon.holes[i], &hole);
    polygon->add_hole(hole);
  }

  if (polygon->outer_boundary().size() < 3) {
    ROS_ERROR_STREAM("Input polygon is not valid.");
    return false;
  } else if (!isStrictlySimple(*polygon)) {
    ROS_ERROR_STREAM("Input polygon is not simple.");
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
      p.x = CGAL::to_double(v.x());
      p.y = CGAL::to_double(v.y());
      p.z = altitude;

      markers->points.push_back(p);
    }
  }
}

}  // namespace polygon_coverage_planning
