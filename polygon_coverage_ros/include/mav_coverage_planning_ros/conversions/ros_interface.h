#ifndef MAV_COVERAGE_PLANNING_ROS_CONVERSIONS_ROS_INTERFACE_H_
#define MAV_COVERAGE_PLANNING_ROS_CONVERSIONS_ROS_INTERFACE_H_

#include <fstream>
#include <vector>

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseArray.h>
#include <mav_msgs/conversions.h>
#include <mav_planning_msgs/Polygon2D.h>
#include <mav_planning_msgs/PolygonWithHolesStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/Marker.h>

#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_visualization/helpers.h>

#include <mav_2d_coverage_planning/geometry/polygon.h>
#include <mav_coverage_planning_comm/cgal_definitions.h>

namespace mav_coverage_planning {

// Warning: Does not set frame or time stamps or orientation.
void eigenTrajectoryPointVectorFromPath(
    const std::vector<Point_2>& waypoints, double altitude,
    mav_msgs::EigenTrajectoryPointVector* traj_points);

void poseArrayMsgFromEigenTrajectoryPointVector(
    const mav_msgs::EigenTrajectoryPointVector& trajectory_points,
    const std::string& frame_id,
    geometry_msgs::PoseArray* trajectory_points_pose_array);

void poseArrayMsgFromPath(
    const std::vector<Point_2>& waypoints, double altitude,
    const std::string& frame_id,
    geometry_msgs::PoseArray* trajectory_points_pose_array);

void msgMultiDofJointTrajectoryFromPath(
    const std::vector<Point_2>& waypoints, double altitude,
    trajectory_msgs::MultiDOFJointTrajectory* msg);

void createMarkers(const std::vector<Point_2>& vertices, double altitude,
                   const std::string& frame_id, const std::string& ns,
                   const mav_visualization::Color& points_color,
                   const mav_visualization::Color& lines_color,
                   const double line_size, const double point_size,
                   visualization_msgs::Marker* points,
                   visualization_msgs::Marker* line_strip);

void createPolygonMarkers(const Polygon& polygon, double altitude,
                          const std::string& frame_id, const std::string& ns,
                          const mav_visualization::Color& polygon_color,
                          const mav_visualization::Color& hole_color,
                          const double line_size, const double point_size,
                          visualization_msgs::MarkerArray* array);

void createStartAndEndPointMarkers(const Point_2& start, const Point_2& end,
                                   double altitude, const std::string& frame_id,
                                   const std::string& ns,
                                   visualization_msgs::Marker* start_point,
                                   visualization_msgs::Marker* end_point);

void createStartAndEndPointMarkers(const mav_msgs::EigenTrajectoryPoint& start,
                                   const mav_msgs::EigenTrajectoryPoint& end,
                                   const std::string& frame_id,
                                   const std::string& ns,
                                   visualization_msgs::Marker* start_point,
                                   visualization_msgs::Marker* end_point);

void createStartAndEndTextMarkers(const Point_2& start, const Point_2& end,
                                  double altitude, const std::string& frame_id,
                                  const std::string& ns,
                                  visualization_msgs::Marker* start_text,
                                  visualization_msgs::Marker* end_text);

void createStartAndEndTextMarkers(const mav_msgs::EigenTrajectoryPoint& start,
                                  const mav_msgs::EigenTrajectoryPoint& end,
                                  const std::string& frame_id,
                                  const std::string& ns,
                                  visualization_msgs::Marker* start_text,
                                  visualization_msgs::Marker* end_text);

bool createPolyhedronMarkerArray(const Polyhedron_3& polyhedron,
                                 const std::string& frame_id,
                                 visualization_msgs::MarkerArray* markers);

void polygon2FromPolygonMsg(const mav_planning_msgs::Polygon2D& msg,
                            Polygon_2* polygon);
bool polygonFromMsg(const mav_planning_msgs::PolygonWithHolesStamped& msg,
                    Polygon* polygon, double* altitude, std::string* frame);

}  // namespace mav_coverage_planning

#endif /* MAV_COVERAGE_PLANNING_ROS_CONVERSIONS_ROS_INTERFACE_H_ */