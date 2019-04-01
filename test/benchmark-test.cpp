#include <iomanip>
#include <sstream>

#include <glog/logging.h>
#include <gtest/gtest.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <mav_planning_msgs/PolygonWithHoles.h>

#include <mav_2d_coverage_planning/geometry/polygon.h>
#include <mav_coverage_planning_comm/cgal_definitions.h>

const std::string kPackageName = "mav_coverage_planning_ros";
const size_t kMaxNoObstacles = 10;
const size_t kNoInstances = 100;

using namespace mav_coverage_planning;

bool loadPolygonFromNode(const YAML::Node& node, Polygon_2* poly) {
  CHECK_NOTNULL(poly);
  if (!node) return false;
  YAML::Node points = node["points"];
  if (!points) return false;
  if (points.size() < 3) return false;
  poly->clear();

  for (size_t i = 0; i < points.size(); ++i) {
    YAML::Node point = points[i];
    if (!point["x"]) return false;
    if (!point["y"]) return false;
    Point_2 p(point["x"].as<double>(), point["y"].as<double>());
    poly->push_back(p);
  }

  return true;
}

bool loadPWHFromFile(const std::string& file, Polygon* polygon) {
  ROS_INFO_STREAM("Loading polygon from " << file);
  YAML::Node node = YAML::LoadFile(file);

  PolygonWithHoles pwh;
  if (!loadPolygonFromNode(node["hull"], &pwh.outer_boundary())) return false;

  for (size_t i = 0; i < node["holes"].size(); ++i) {
    Polygon_2 poly;
    if (!loadPolygonFromNode(node["holes"][i], &poly)) return false;
    pwh.add_hole(poly);
  }

  ROS_INFO_STREAM("Successfully loaded PWH with " << pwh.number_of_holes()
                                                  << " holes.");

  CHECK_NOTNULL(polygon);
  *polygon = Polygon(pwh);

  return true;
}

bool loadAllInstances(std::vector<std::vector<Polygon>>* polys) {
  CHECK_NOTNULL(polys);
  polys->resize(kMaxNoObstacles, std::vector<Polygon>(kNoInstances));

  std::string instances_path = ros::package::getPath(kPackageName);
  instances_path = instances_path.substr(0, instances_path.find("/src/"));
  instances_path +=
      "/build/" + kPackageName + "/pwh_instances-prefix/src/pwh_instances/";

  for (size_t i = 0; i < kMaxNoObstacles; ++i) {
    std::string subfolder = instances_path + std::to_string(i) + "/";
    for (size_t j = 0; j < kNoInstances; ++j) {
      std::stringstream ss;
      ss << std::setw(4) << std::setfill('0') << j;
      if (!loadPWHFromFile(subfolder + ss.str() + ".yaml", &(*polys)[i][j]))
        return false;
    }
  }

  return true;
}

TEST(BenchmarkTest, Benchmark) {
  std::vector<std::vector<Polygon>> polys;
  // Load polygons.
  CHECK(loadAllInstances(&polys));

}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  return RUN_ALL_TESTS();
}
