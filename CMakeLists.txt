cmake_minimum_required(VERSION 2.8.4)
project(mav_coverage_graph_solvers)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

##########
# EXPORT #
##########
cs_install()
cs_export()
