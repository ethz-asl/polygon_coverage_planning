cmake_minimum_required(VERSION 2.8.4)
project(polygon_coverage_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17)

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
  URL_MD5 87b964a974f6a8dd174dfe393b631a61
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  #src/coverage_planner.cc
  src/polygon_planner_base.cc
  src/ros_interface.cc
  #test/test_polygon_coverage_planning.cc
)
target_link_libraries(${PROJECT_NAME})

############
# Binaries #
############
#cs_add_executable(polygon_coverage_planner
#  src/polygon_coverage_planner.cc
#)
#target_link_libraries(polygon_coverage_planner ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
