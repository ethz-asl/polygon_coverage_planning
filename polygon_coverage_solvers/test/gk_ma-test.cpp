#include <limits>
#include <random>
#include <vector>

#include <gtest/gtest.h>
#include <ros/package.h>

#include "polygon_coverage_solvers/gk_ma.h"

using namespace mav_coverage_planning;
using namespace gk_ma;

const std::string kPackageName = "polygon_coverage_solvers";

TEST(GkMa, LoadFromFile) {
  GkMa& instance = GkMa::getInstance();

  // Package directory.
  std::string instances_path = ros::package::getPath(kPackageName);
  // Catkin directory.
  instances_path = instances_path.substr(0, instances_path.find("/src/"));
  // Instances directory.
  instances_path +=
      "/build/" + kPackageName + "/gtsp_instances-prefix/src/gtsp_instances/";

  std::vector<std::string> instance_names = {"4br17.gtsp", "11berlin52.gtsp",
                                             "40d198.gtsp", "65rbg323.gtsp"};
  for (const std::string& instance_name : instance_names) {
    std::string file = instances_path + instance_name;
    instance.setSolver(file, true);
    EXPECT_TRUE(instance.solve());
    EXPECT_FALSE(instance.getSolution().empty());
  }
}

TEST(GkMa, LoadFromTask) {
  GkMa& instance = GkMa::getInstance();

  std::srand(123456);

  std::vector<std::vector<int>> m(10, std::vector<int>(10));
  for (size_t i = 0; i < m.size(); ++i) {
    for (size_t j = 0; j < m[i].size(); ++j) {
      if (i == j) {
        m[i][j] = std::numeric_limits<int>::max();
      } else {
        m[i][j] = rand() % 100;
      }
    }
  }

  std::vector<std::vector<int>> clusters(4);
  clusters[0].push_back(0);
  clusters[1].push_back(1);
  clusters[1].push_back(2);
  clusters[1].push_back(3);
  clusters[1].push_back(4);
  clusters[1].push_back(5);
  clusters[2].push_back(6);
  clusters[2].push_back(7);
  clusters[2].push_back(8);
  clusters[3].push_back(9);

  Task task(m, clusters);
  EXPECT_FALSE(task.mIsSymmetric());

  instance.setSolver(task);
  EXPECT_TRUE(instance.solve());
  std::vector<int> solution = instance.getSolution();
  EXPECT_EQ(solution.size(), clusters.size());
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
