cmake_minimum_required(VERSION 2.8.4)
project(polygon_coverage_geometry)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(CGAL QUIET COMPONENTS Core)
include(${CGAL_USE_FILE})

add_definitions(-std=c++11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_BUILD_TYPE Release)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  #src/geometry/plane_transformation.cc
  #src/geometry/polygon_triangulation.cc
  #src/geometry/polygon.cc
  #src/geometry/BCD.cpp
  src/bcd.cc
  src/cgal_comm.cc
  src/visibility_polygon.cc
  src/weakly_monotone.cc
  #src/geometry/sweep.cc
)
target_link_libraries(${PROJECT_NAME} ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})

#########
# TESTS #
#########
#catkin_add_gtest(test_polygon
#  test/polygon-test.cpp
#)
#target_link_libraries(test_polygon ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
