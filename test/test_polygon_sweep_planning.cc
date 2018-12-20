#include <mav_2d_coverage_planning/geometry/polygon.h>
#include <mav_coverage_planning_ros/conversions/msg_from_xml_rpc.h>
#include <mav_coverage_planning_ros/conversions/ros_interface.h>
#include <mav_planning_msgs/PolygonWithHolesStamped.h>
#include <ros/ros.h>

using namespace mav_coverage_planning;

int main(int argc, char** argv) {
  ros::init(argc, argv, "sweep_calculator");

  ros::NodeHandle nh_private("~");
  // Load polygon.
  XmlRpc::XmlRpcValue polygon_xml_rpc;
  const std::string polygon_param_name = "polygon";
  if (!nh_private.getParam(polygon_param_name, polygon_xml_rpc)) {
    ROS_ERROR("No polygon set on parameter server.");
    return 0;
  }
  mav_planning_msgs::PolygonWithHolesStamped poly_msg;
  Polygon poly;
  double altitude;
  std::string global_frame_id;
  if (!PolygonWithHolesStampedMsgFromXmlRpc(polygon_xml_rpc, &poly_msg)) {
    ROS_ERROR("Failed to get polygon from XMLRpc.");
    return 0;
  }
  if (polygonFromMsg(poly_msg, &poly, &altitude, &global_frame_id)) {
    ROS_INFO_STREAM("Successfully loaded polygon.");
    ROS_INFO_STREAM("Altiude: " << altitude << "m");
    ROS_INFO_STREAM("Global frame: " << global_frame_id);
    ROS_INFO_STREAM("Polygon:" << poly);
  } else {
    ROS_WARN_STREAM("Failed reading polygon message from parameter server.");
  }
  // Publish polygon.
  ros::Publisher pub = nh_private.advertise<visualization_msgs::MarkerArray>(
      "visualization", 1, true);
  visualization_msgs::MarkerArray polygon_markers;
  createPolygonMarkers(poly, altitude, global_frame_id, "polygon",
                       mav_visualization::Color::Blue(),
                       mav_visualization::Color::Orange(), &polygon_markers);

  // Publish waypoint list.
  ros::Publisher waypoints_pub =
      nh_private.advertise<visualization_msgs::MarkerArray>("waypoints", 1,
                                                            true);

  ros::Rate loop_rate(1.0);
  size_t start_id = 0;
  bool counter_clockwise = true;
  while (ros::ok()) {
    std::vector<Point_2> waypoints;
    const double kMaxSweepDistance = 1.0;
    ROS_INFO_STREAM("Computing sweep plan for start_id: "
                    << start_id
                    << " and counter_clockwise: " << counter_clockwise);
    bool success = poly.computeLineSweepPlan(kMaxSweepDistance, start_id,
                                             counter_clockwise, &waypoints);
    counter_clockwise = !counter_clockwise;
    if (counter_clockwise) {
      start_id = (start_id + 1) % poly.getPolygon().outer_boundary().size();
    }
    if (!success) {
      ROS_WARN_STREAM("Failed to compute sweep plan");
      continue;
    }

    // The planned path:
    visualization_msgs::MarkerArray all_markers;
    visualization_msgs::Marker path_points, path_line_strips;
    createMarkers(waypoints, altitude, global_frame_id, "vertices_and_strip",
                  mav_visualization::Color::Red(),
                  mav_visualization::Color::Green(), &path_points,
                  &path_line_strips);
    all_markers.markers.push_back(path_points);
    all_markers.markers.push_back(path_line_strips);

    // Start and end points
    visualization_msgs::Marker start_point, end_point;
    createStartAndEndPointMarkers(waypoints.front(), waypoints.back(), altitude,
                                  global_frame_id, "points", &start_point,
                                  &end_point);
    all_markers.markers.push_back(start_point);
    all_markers.markers.push_back(end_point);

    all_markers.markers.insert(all_markers.markers.end(),
                               polygon_markers.markers.begin(),
                               polygon_markers.markers.end());
    pub.publish(all_markers);
    loop_rate.sleep();
  }

  return 0;
}
