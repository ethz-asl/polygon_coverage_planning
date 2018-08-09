#ifndef MAV_COVERAGE_PLANNING_ROS_INTERFACE_H_
#define MAV_COVERAGE_PLANNING_ROS_INTERFACE_H_

#include <fstream>
#include <vector>

#include <Eigen/Core>

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseArray.h>
#include <mav_msgs/conversions.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <visualization_msgs/Marker.h>
#include <planning_msgs/Polygon2D.h>
#include <planning_msgs/PolygonWithHolesStamped.h>

#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_visualization/helpers.h>

#include "mav_coverage_planning/common.h"
#include "mav_coverage_planning/polygon.h"

namespace mav_coverage_planning {

// Warning: Does not set frame or time stamps or orientation.
void eigenTrajectoryPointVectorFromPath(
    const StdVector2d& waypoints, double altitude,
    mav_msgs::EigenTrajectoryPointVector* traj_points);

void poseArrayMsgFromEigenTrajectoryPointVector(
    const mav_msgs::EigenTrajectoryPointVector& trajectory_points,
    const std::string& frame_id,
    geometry_msgs::PoseArray* trajectory_points_pose_array);

void poseArrayMsgFromPath(
    const StdVector2d& waypoints, double altitude, const std::string& frame_id,
    geometry_msgs::PoseArray* trajectory_points_pose_array);

void msgMultiDofJointTrajectoryFromPath(
    const StdVector2d& waypoints, double altitude,
    trajectory_msgs::MultiDOFJointTrajectory* msg);

void createMarkers(const StdVector2d& vertices, double altitude,
                   const std::string& frame_id, const std::string& ns,
                   const mav_visualization::Color& points_color,
                   const mav_visualization::Color& lines_color,
                   visualization_msgs::Marker* points,
                   visualization_msgs::Marker* line_strip);

void createPolygonMarkers(const Polygon& polygon, double altitude,
                          const std::string& frame_id, const std::string& ns,
                          const mav_visualization::Color& polygon_color,
                          const mav_visualization::Color& hole_color,
                          visualization_msgs::MarkerArray* array);

void createMarkers(const StdVector2d& vertices, double altitude,
                   visualization_msgs::Marker* points,
                   visualization_msgs::Marker* line_strip);

void createStartAndEndPointMarkers(const Eigen::Vector2d& start,
                                   const Eigen::Vector2d& end, double altitude,
                                   const std::string& frame_id,
                                   const std::string& ns,
                                   visualization_msgs::Marker* start_point,
                                   visualization_msgs::Marker* end_point);

void verticesFromPolygonMsg(const planning_msgs::Polygon2D& msg,
                            StdVector2d* vertices);
bool polygonFromMsg(const planning_msgs::PolygonWithHolesStamped& msg,
                    Polygon* polygon, double* altitude, std::string* frame);

}  // namespace mav_coverage_planning

#endif /* MAV_COVERAGE_PLANNING_ROS_INTERFACE_H_ */
