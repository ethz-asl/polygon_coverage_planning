#include <iomanip>
#include <sstream>

#include <glog/logging.h>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <mav_planning_msgs/PolygonWithHoles.h>

const std::string kPackageName = "mav_coverage_planning_ros";
const size_t kMaxNoObstacles = 10;

bool loadPolygonFromNode(const YAML::Node& node,
                         mav_planning_msgs::Polygon2D* poly) {
  CHECK_NOTNULL(poly);
  if (!node) return false;
  YAML::Node points = node["points"];
  if (!points) return false;
  if (points.size() < 3) return false;
  poly->points.clear();

  for (size_t i = 0; i < points.size(); ++i) {
    YAML::Node point = points[i];
    if (!point["x"]) return false;
    if (!point["y"]) return false;
    mav_planning_msgs::Point2D p;
    p.x = point["x"].as<double>();
    p.y = point["y"].as<double>();
    poly->points.push_back(p);
  }

  return true;
}

bool loadPWHFromFile(const std::string& file) {
  ROS_INFO_STREAM("Loading polygon from " << file);
  YAML::Node node = YAML::LoadFile(file);

  mav_planning_msgs::PolygonWithHoles pwh;
  if (!loadPolygonFromNode(node["hull"], &pwh.hull)) return false;

  pwh.holes.clear();
  for (size_t i = 0; i < node["holes"].size(); ++i) {
    mav_planning_msgs::Polygon2D poly;
    if (!loadPolygonFromNode(node["holes"][i], &poly)) return false;
    pwh.holes.push_back(poly);
  }

  ROS_INFO_STREAM("Successfully loaded PWH with " << pwh.holes.size()
                                                  << " holes.");

  return true;
}

TEST(BenchmarkTest, Benchmark) {
  // Load polygons.
  std::string instances_path = ros::package::getPath(kPackageName);
  instances_path = instances_path.substr(0, instances_path.find("/src/"));
  instances_path +=
      "/build/" + kPackageName + "/pwh_instances-prefix/src/pwh_instances/";

  for (size_t i = 0; i < kMaxNoObstacles; ++i) {
    std::string subfolder = instances_path + std::to_string(i) + "/";
    for (size_t j = 0; j < 100; ++j) {
      std::stringstream ss;
      ss << std::setw(4) << std::setfill('0') << j;
      CHECK(loadPWHFromFile(subfolder + ss.str() + ".yaml"));
    }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  return RUN_ALL_TESTS();
}
