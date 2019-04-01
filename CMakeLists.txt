cmake_minimum_required(VERSION 2.8.4)
project(mav_coverage_planning_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(CGAL QUIET COMPONENTS Core)
include(${CGAL_USE_FILE})

add_definitions(-std=c++11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

############
# YAML-CPP #
############
# Link against system catkin yaml-cpp if installed.
find_package(PkgConfig)
find_package(yaml_cpp_catkin QUIET)
if(${yaml_cpp_catkin_FOUND})
    message(STATUS "Found yaml_cpp_catkin, using instead of system library.")
    set(YamlCpp_LIBRARIES ${yaml_cpp_catkin_LIBRARIES})
    set(YamlCpp_INCLUDE_DIRS ${yaml_cpp_catkin_INCLUDE_DIRS})
else()
    message(STATUS "No yaml_cpp_catkin, using yaml-cpp system library instead.")
    pkg_check_modules(YamlCpp REQUIRED yaml-cpp>=0.5)
endif()

# Download test instances.
include(ExternalProject)
ExternalProject_Add(
  pwh_instances
  URL https://polybox.ethz.ch/index.php/s/KzD3L3U5KXdd8Mz/download
  DOWNLOAD_NAME pwh.tar.gz
  URL_MD5 fa655197c0ceffb004772c259d0ecaa8
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/nodes/base_planner.cc
  src/nodes/2d/base_planner_2d.cc
  src/nodes/2d/shortest_path_2d.cc
  src/conversions/ros_interface.cc
  test/test_polygon_sweep_planning.cc
)
target_link_libraries(${PROJECT_NAME} ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})

############
# Binaries #
############
cs_add_executable(stripmap_planner_2d_node
  src/nodes/2d/stripmap_planner_2d_node.cc
)
target_link_libraries(stripmap_planner_2d_node ${PROJECT_NAME})

cs_add_executable(stripmap_planner_2d_exact_node
  src/nodes/2d/stripmap_planner_2d_exact_node.cc
)
target_link_libraries(stripmap_planner_2d_exact_node ${PROJECT_NAME})

cs_add_executable(stripmap_planner_2d_exact_preprocessed_node
  src/nodes/2d/stripmap_planner_2d_exact_preprocessed_node.cc
)
target_link_libraries(stripmap_planner_2d_exact_preprocessed_node ${PROJECT_NAME})

cs_add_executable(shortest_path_2d_node
  src/nodes/2d/shortest_path_2d_node.cc
)
target_link_libraries(shortest_path_2d_node ${PROJECT_NAME})

cs_add_executable(stripmap_planner_3d_node
  src/nodes/3d/stripmap_planner_3d_node.cc
)
target_link_libraries(stripmap_planner_3d_node ${PROJECT_NAME})

cs_add_executable(test_polygon_sweep_planning
  test/test_polygon_sweep_planning.cc
)
target_link_libraries(test_polygon_sweep_planning ${PROJECT_NAME})

#########
# TESTS #
#########
catkin_add_gtest(test_benchmark
  test/benchmark-test.cpp
)
target_link_libraries(test_benchmark ${PROJECT_NAME} ${YamlCpp_LIBRARIES})

##########
# EXPORT #
##########
cs_install()
cs_export()
