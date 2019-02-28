#include "mav_coverage_planning_ros/conversions/ros_interface.h"

#include <algorithm>
#include <limits>

#include <geometry_msgs/Point.h>
#include <glog/logging.h>
#include <mav_coverage_planning_comm/cgal_definitions.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>

namespace mav_coverage_planning {

void eigenTrajectoryPointVectorFromPath(
    const std::vector<Point_2>& waypoints, double altitude,
    mav_msgs::EigenTrajectoryPointVector* traj_points) {
  CHECK_NOTNULL(traj_points);

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
    const std::vector<Point_2>& waypoints, double altitude,
    const std::string& frame_id,
    geometry_msgs::PoseArray* trajectory_points_pose_array) {
  CHECK_NOTNULL(trajectory_points_pose_array);

  mav_msgs::EigenTrajectoryPointVector eigen_traj;
  eigenTrajectoryPointVectorFromPath(waypoints, altitude, &eigen_traj);
  poseArrayMsgFromEigenTrajectoryPointVector(eigen_traj, frame_id,
                                             trajectory_points_pose_array);
}

void msgMultiDofJointTrajectoryFromPath(
    const std::vector<Point_2>& waypoints, double altitude,
    trajectory_msgs::MultiDOFJointTrajectory* msg) {
  CHECK_NOTNULL(msg);

  mav_msgs::EigenTrajectoryPointVector eigen_traj;
  eigenTrajectoryPointVectorFromPath(waypoints, altitude, &eigen_traj);
  mav_msgs::msgMultiDofJointTrajectoryFromEigen(eigen_traj, msg);
}

void createMarkers(const std::vector<Point_2>& vertices, double altitude,
                   const std::string& frame_id, const std::string& ns,
                   const mav_visualization::Color& points_color,
                   const mav_visualization::Color& lines_color,
                   const double line_size,
                   const double point_size,
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

void createPolygonMarkers(const Polygon& polygon, double altitude,
                          const std::string& frame_id, const std::string& ns,
                          const mav_visualization::Color& polygon_color,
                          const mav_visualization::Color& hole_color,
                          const double line_size, const double point_size,
                          visualization_msgs::MarkerArray* array) {
  CHECK_NOTNULL(array);
  array->markers.clear();

  // Polygon markers.
  visualization_msgs::Marker hull_points, hull_vertices;
  // Hull.
  std::vector<Point_2> hull = polygon.getHullVertices();
  hull.push_back(hull.front());
  createMarkers(hull, altitude, frame_id, ns + "hull", polygon_color,
                polygon_color, line_size, point_size, &hull_points,
                &hull_vertices);
  array->markers.push_back(hull_points);
  array->markers.push_back(hull_vertices);

  // Hole markers:
  std::vector<std::vector<Point_2>> holes = polygon.getHoleVertices();
  size_t i = 0;
  for (std::vector<Point_2>& hole : holes) {
    visualization_msgs::Marker hole_points, hole_vertices;
    hole.push_back(hole.front());
    createMarkers(hole, altitude, frame_id, ns + "hole_" + std::to_string(i++),
                  hole_color, hole_color, line_size, point_size, &hole_points,
                  &hole_vertices);
    array->markers.push_back(hole_points);
    array->markers.push_back(hole_vertices);
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
  CHECK_NOTNULL(start_point);
  CHECK_NOTNULL(end_point);

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

  start_point->color = mav_visualization::Color::Green();
  start_point->color.a = 0.5;

  end_point->color = mav_visualization::Color::Red();
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
  CHECK_NOTNULL(start_text);
  CHECK_NOTNULL(end_text);

  start_text->header.frame_id = end_text->header.frame_id = frame_id;
  start_text->header.stamp = end_text->header.stamp = ros::Time::now();
  start_text->ns = ns + "_start_text";
  start_text->ns = ns + "_end_text";
  start_text->action = end_text->action = visualization_msgs::Marker::ADD;
  start_text->type = end_text->type = visualization_msgs::Marker::TEXT_VIEW_FACING;

  geometry_msgs::PoseStamped start_stamped, end_stamped;
  msgPoseStampedFromEigenTrajectoryPoint(start, &start_stamped);
  msgPoseStampedFromEigenTrajectoryPoint(end, &end_stamped);

  start_text->pose = start_stamped.pose;
  end_text->pose = end_stamped.pose;

  start_text->pose.orientation.w = end_text->pose.orientation.w = 1.0;

  start_text->text = "S";
  start_text->color = mav_visualization::Color::Black();

  end_text->text = "G";
  end_text->color = mav_visualization::Color::Black();

  start_text->scale.z = end_text->scale.z = 1.0;
}

void polygon2FromPolygonMsg(const mav_planning_msgs::Polygon2D& msg,
                            Polygon_2* polygon) {
  CHECK_NOTNULL(polygon);

  std::vector<Point_2> vertices(msg.points.size());
  for (size_t i = 0; i < msg.points.size(); ++i)
    vertices[i] = Point_2(msg.points[i].x, msg.points[i].y);

  *polygon = Polygon_2(vertices.begin(), vertices.end());
}

bool polygonFromMsg(const mav_planning_msgs::PolygonWithHolesStamped& msg,
                    Polygon* polygon, double* altitude, std::string* frame) {
  CHECK_NOTNULL(polygon);
  CHECK_NOTNULL(altitude);
  CHECK_NOTNULL(frame);

  *frame = msg.header.frame_id;
  *altitude = msg.altitude;

  Polygon_2 hull;
  polygon2FromPolygonMsg(msg.polygon.hull, &hull);
  PolygonWithHoles pwh(hull);

  for (size_t i = 0; i < msg.polygon.holes.size(); ++i) {
    Polygon_2 hole;
    polygon2FromPolygonMsg(msg.polygon.holes[i], &hole);
    pwh.add_hole(hole);
  }

  *polygon = Polygon(pwh);
  if (polygon->getPolygon().outer_boundary().size() < 3) {
    ROS_ERROR_STREAM("Input polygon is not valid.");
    return false;
  } else if (!polygon->isStrictlySimple()) {
    ROS_ERROR_STREAM("Input polygon is not simple.");
    return false;
  }
  return true;
}

bool createPolyhedronMarkerArray(const Polyhedron_3& polyhedron,
                                 const std::string& frame_id,
                                 visualization_msgs::MarkerArray* markers) {
  typedef Polyhedron_3::Halfedge_around_facet_circulator halfedge_circulator;
  typedef boost::graph_traits<Polyhedron_3>::face_descriptor face_descriptor;

  CHECK_NOTNULL(markers);
  markers->markers.clear();

  // Triangulate mesh.
  Polyhedron_3 mesh = polyhedron;
  if (!CGAL::Polygon_mesh_processing::triangulate_faces(mesh)) return false;

  // Create faces.
  visualization_msgs::Marker triangle_list;
  triangle_list.header.frame_id = frame_id;
  triangle_list.header.stamp = ros::Time::now();
  triangle_list.ns = "faces";
  triangle_list.id = 0;
  triangle_list.type = visualization_msgs::Marker::TRIANGLE_LIST;
  triangle_list.action = visualization_msgs::Marker::ADD;
  triangle_list.pose.position.x = 0.0;
  triangle_list.pose.position.y = 0.0;
  triangle_list.pose.position.z = 0.0;
  triangle_list.pose.orientation.x = 0.0;
  triangle_list.pose.orientation.y = 0.0;
  triangle_list.pose.orientation.z = 0.0;
  triangle_list.pose.orientation.w = 1.0;
  triangle_list.scale.x = 1.0;
  triangle_list.scale.y = 1.0;
  triangle_list.scale.z = 1.0;
  triangle_list.color.r = 0.5;
  triangle_list.color.g = 0.5;
  triangle_list.color.b = 0.5;
  triangle_list.color.a = 1.0;
  for (const face_descriptor& f : faces(mesh)) {
    halfedge_circulator he_c = f->facet_begin();
    do {
      geometry_msgs::Point p;
      p.x = CGAL::to_double(he_c->vertex()->point().x());
      p.y = CGAL::to_double(he_c->vertex()->point().y());
      p.z = CGAL::to_double(he_c->vertex()->point().z());
      triangle_list.points.push_back(p);
    } while (++he_c != f->facet_begin());
  }
  markers->markers.push_back(triangle_list);

  // Create lines.
  size_t id = 0;
  for (const face_descriptor& f : faces(mesh)) {
    visualization_msgs::Marker border;
    border.header.frame_id = frame_id;
    border.header.stamp = ros::Time::now();
    border.ns = "border_" + std::to_string(id);
    border.id = id++;
    border.type = visualization_msgs::Marker::LINE_STRIP;
    border.action = visualization_msgs::Marker::ADD;
    border.pose.position.x = 0.0;
    border.pose.position.y = 0.0;
    border.pose.position.z = 0.0;
    border.pose.orientation.x = 0.0;
    border.pose.orientation.y = 0.0;
    border.pose.orientation.z = 0.0;
    border.pose.orientation.w = 1.0;
    border.scale.x = 0.1;
    border.color.r = 0.0;
    border.color.g = 0.0;
    border.color.b = 0.0;
    border.color.a = 1.0;

    halfedge_circulator he_c = f->facet_begin();
    do {
      geometry_msgs::Point p;
      p.x = CGAL::to_double(he_c->vertex()->point().x());
      p.y = CGAL::to_double(he_c->vertex()->point().y());
      p.z = CGAL::to_double(he_c->vertex()->point().z());
      border.points.push_back(p);
    } while (++he_c != f->facet_begin());
    geometry_msgs::Point p;
    p.x = CGAL::to_double(he_c->vertex()->point().x());
    p.y = CGAL::to_double(he_c->vertex()->point().y());
    p.z = CGAL::to_double(he_c->vertex()->point().z());
    border.points.push_back(p);

    markers->markers.push_back(border);
  }

  return true;
}

}  // namespace mav_coverage_planning
