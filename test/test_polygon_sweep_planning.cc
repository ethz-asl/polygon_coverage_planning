#include <mav_2d_coverage_planning/geometry/polygon.h>
#include <mav_coverage_planning_ros/conversions/msg_from_xml_rpc.h>
#include <mav_coverage_planning_ros/conversions/ros_interface.h>
#include <mav_planning_msgs/PolygonWithHolesStamped.h>
#include <ros/ros.h>

#include <limits>
#include <mav_coverage_planning_comm/cgal_definitions.h>

using namespace mav_coverage_planning;

bool computeLineSweepPlans(const Polygon& polygon,
                           std::vector<std::vector<Point_2>>* cluster_sweeps) {
  CHECK_NOTNULL(cluster_sweeps);
  cluster_sweeps->clear();
  cluster_sweeps->reserve(2 * polygon.getPolygon().outer_boundary().size());

  // Create all sweep plans.
  bool cc_orientation = true;
  std::vector<EdgeConstIterator> dirs_swept;
  for (size_t start_id = 0;
       start_id < polygon.getPolygon().outer_boundary().size(); ++start_id) {
    // Don't sweep same direction multiple times.
    EdgeConstIterator dir = std::next(
        polygon.getPolygon().outer_boundary().edges_begin(), start_id);
    std::vector<EdgeConstIterator>::iterator it =
        std::find_if(dirs_swept.begin(), dirs_swept.end(),
                     [&dir](const EdgeConstIterator& dir_swept) {
                       return CGAL::parallel(*dir, *dir_swept);
                     });
    if (it != dirs_swept.end()) {
      DLOG(INFO) << "Direction already swept.";
      continue;
    }
    dirs_swept.push_back(dir);

    // Create 4 sweeps. Along direction, along opposite direction and reverse.
    std::vector<Point_2> sweep;
    const double kMaxSweepDistance = 4.0;
    if (!polygon.computeLineSweepPlan(kMaxSweepDistance, start_id,
                                      cc_orientation, &sweep)) {
      LOG(WARNING)
          << "Could not compute counter clockwise sweep plan for start_id: "
          << start_id << " in polygon: " << polygon;
    } else {
      CHECK(!sweep.empty());
      cluster_sweeps->push_back(sweep);
      std::reverse(sweep.begin(), sweep.end());
      cluster_sweeps->push_back(sweep);
    }

    if (!polygon.computeLineSweepPlan(
            kMaxSweepDistance,
            (start_id + 1) % polygon.getPolygon().outer_boundary().size(),
            !cc_orientation, &sweep)) {
      LOG(WARNING) << "Could not compute clockwise sweep plan for start_id: "
                   << start_id << " in polygon: " << polygon;
    } else {
      CHECK(!sweep.empty());
      cluster_sweeps->push_back(sweep);
      std::reverse(sweep.begin(), sweep.end());
      cluster_sweeps->push_back(sweep);
    }
  }

  return true;
}

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
  const double kPolygonLineSize = 0.4;
  const double kPolygonPointSize = 0.4;
  createPolygonMarkers(poly, altitude, global_frame_id, "polygon",
                       mav_visualization::Color::Black(),
                       mav_visualization::Color::Black(), kPolygonLineSize,
                       kPolygonPointSize, &polygon_markers);

  // Publish waypoint list.
  ros::Publisher waypoints_pub =
      nh_private.advertise<visualization_msgs::MarkerArray>("waypoints", 1,
                                                            true);

  // Compute sweep permutations.
  std::vector<std::vector<Point_2>> waypoints;
  computeLineSweepPlans(poly, &waypoints);

  ros::Rate loop_rate(1.0);
  size_t i = 0;
  while (ros::ok()) {
    // The planned path:
    visualization_msgs::MarkerArray all_markers;
    visualization_msgs::Marker path_points, path_line_strips;
    const double kPathLineSize = 0.2;
    const double kPathPointSize = 0.4;
    createMarkers(waypoints[i], altitude, global_frame_id, "vertices_and_strip",
                  mav_visualization::Color::Gray(),
                  mav_visualization::Color::Gray(), kPathLineSize,
                  kPathPointSize, &path_points, &path_line_strips);
    all_markers.markers.push_back(path_points);
    all_markers.markers.push_back(path_line_strips);

    // Start and end points
    visualization_msgs::Marker start_point, end_point;
    createStartAndEndPointMarkers(waypoints[i].front(), waypoints[i].back(),
                                  altitude, global_frame_id, "points",
                                  &start_point, &end_point);
    all_markers.markers.push_back(start_point);
    all_markers.markers.push_back(end_point);

    // Start and end text.
    visualization_msgs::Marker start_text, end_text;
    createStartAndEndTextMarkers(waypoints[i].front(), waypoints[i].back(),
                                 altitude, global_frame_id, "text",
                                 &start_text, &end_text);
    all_markers.markers.push_back(start_text);
    all_markers.markers.push_back(end_text);

    // Start arrow.
    visualization_msgs::Marker start_arrow;
    start_arrow.type = visualization_msgs::Marker::ARROW;
    start_arrow.action = visualization_msgs::Marker::ADD;
    start_arrow.points.push_back(start_point.pose.position);
    geometry_msgs::Point end;
    end.x = CGAL::to_double(std::next(waypoints[i].begin())->x());
    end.y = CGAL::to_double(std::next(waypoints[i].begin())->y());
    end.z = altitude;
    start_arrow.points.push_back(end);
    start_arrow.color = mav_visualization::Color::Red();
    start_arrow.scale.x = 0.8;
    start_arrow.scale.y = 2.0;
    start_arrow.scale.z = 3.0;
    start_arrow.header.frame_id = global_frame_id;
    start_arrow.header.stamp = ros::Time::now();
    start_arrow.ns = "dir";
    all_markers.markers.push_back(start_arrow);

    all_markers.markers.insert(all_markers.markers.end(),
                               polygon_markers.markers.begin(),
                               polygon_markers.markers.end());
    pub.publish(all_markers);

    i++;
    i = i % waypoints.size();

    std::cout << "Press ENTER to continue...";
    std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
  }

  return 0;
}
