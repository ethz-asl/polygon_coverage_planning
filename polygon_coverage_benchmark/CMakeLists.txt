cmake_minimum_required(VERSION 2.8.3)
project(polygon_coverage_benchmark)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

catkin_python_setup()
cs_install()
cs_export()
