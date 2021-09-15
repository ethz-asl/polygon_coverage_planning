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

#ifndef POLYGON_COVERAGE_ROS_INTERFACE_H_
#define POLYGON_COVERAGE_ROS_INTERFACE_H_

#include <fstream>
#include <vector>

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <polygon_coverage_msgs/PolygonWithHolesStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <polygon_coverage_geometry/cgal_definitions.h>

namespace polygon_coverage_planning {

class Color : public std_msgs::ColorRGBA {
 public:
  Color() : std_msgs::ColorRGBA() {}
  Color(double red, double green, double blue) : Color(red, green, blue, 1.0) {}
  Color(double red, double green, double blue, double alpha) : Color() {
    r = red;
    g = green;
    b = blue;
    a = alpha;
  }

  static const Color White() { return Color(1.0, 1.0, 1.0); }
  static const Color Black() { return Color(0.0, 0.0, 0.0); }
  static const Color Gray() { return Color(0.5, 0.5, 0.5); }
  static const Color Red() { return Color(1.0, 0.0, 0.0); }
  static const Color Green() { return Color(0.0, 1.0, 0.0); }
  static const Color Blue() { return Color(0.0, 0.0, 1.0); }
  static const Color Yellow() { return Color(1.0, 1.0, 0.0); }
  static const Color Orange() { return Color(1.0, 0.5, 0.0); }
  static const Color Purple() { return Color(0.5, 0.0, 1.0); }
  static const Color Chartreuse() { return Color(0.5, 1.0, 0.0); }
  static const Color Teal() { return Color(0.0, 1.0, 1.0); }
  static const Color Pink() { return Color(1.0, 0.0, 0.5); }
};

// Warning: Does not set frame or time stamps or orientation.
void poseArrayMsgFromPath(
    const std::vector<Point_2>& waypoints, double altitude,
    const std::string& frame_id,
    geometry_msgs::PoseArray* trajectory_points_pose_array);

void msgMultiDofJointTrajectoryFromPath(
    const std::vector<Point_2>& waypoints, double altitude,
    trajectory_msgs::MultiDOFJointTrajectory* msg);

void msgPointFromWaypoint(const Point_2& waypoint, double altitude,
                          geometry_msgs::Point* point);

void createMarkers(const std::vector<Point_2>& vertices, double altitude,
                   const std::string& frame_id, const std::string& ns,
                   const Color& points_color, const Color& lines_color,
                   const double line_size, const double point_size,
                   visualization_msgs::Marker* points,
                   visualization_msgs::Marker* line_strip);

void createTriangles(const std::vector<std::vector<Point_2>>& triangles,
                     const std::string& frame_id, const std::string& ns,
                     const Color& color, const double altitude,
                     visualization_msgs::Marker* markers);

void createPolygonMarkers(const PolygonWithHoles& polygon, double altitude,
                          const std::string& frame_id, const std::string& ns,
                          const Color& polygon_color, const Color& hole_color,
                          const double line_size, const double point_size,
                          visualization_msgs::MarkerArray* array);

void createStartAndEndPointMarkers(const Point_2& start, const Point_2& end,
                                   double altitude, const std::string& frame_id,
                                   const std::string& ns,
                                   visualization_msgs::Marker* start_point,
                                   visualization_msgs::Marker* end_point);

void createStartAndEndPointMarkers(const geometry_msgs::Pose& start,
                                   const geometry_msgs::Pose& end,
                                   const std::string& frame_id,
                                   const std::string& ns,
                                   visualization_msgs::Marker* start_point,
                                   visualization_msgs::Marker* end_point);

void createStartAndEndTextMarkers(const Point_2& start, const Point_2& end,
                                  double altitude, const std::string& frame_id,
                                  const std::string& ns,
                                  visualization_msgs::Marker* start_text,
                                  visualization_msgs::Marker* end_text);

void createStartAndEndTextMarkers(const geometry_msgs::Pose& start,
                                  const geometry_msgs::Pose& end,
                                  const std::string& frame_id,
                                  const std::string& ns,
                                  visualization_msgs::Marker* start_text,
                                  visualization_msgs::Marker* end_text);

void polygon2FromPolygonMsg(const geometry_msgs::Polygon& msg,
                            Polygon_2* polygon);
bool polygonFromMsg(const polygon_coverage_msgs::PolygonWithHolesStamped& msg,
                    PolygonWithHoles* polygon, double* altitude,
                    std::string* frame);

}  // namespace polygon_coverage_planning

#endif /* POLYGON_COVERAGE_ROS_INTERFACE_H_ */
