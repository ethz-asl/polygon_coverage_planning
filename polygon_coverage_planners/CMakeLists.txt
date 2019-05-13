cmake_minimum_required(VERSION 2.8.4)
project(polygon_coverage_planners)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(CGAL QUIET COMPONENTS Core)
include(${CGAL_USE_FILE})

add_definitions(-std=c++11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_BUILD_TYPE Release)

# TODO(rikba): Make catkin package.
find_package(PkgConfig)
pkg_check_modules(MONO mono-2 REQUIRED)
include_directories(${MONO_INCLUDE_DIRS})

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/cost_functions/path_cost_functions.cc
  #src/graphs/gtspp_product_graph.cc
  #src/graphs/sweep_plan_graph.cc
  #src/graphs/visibility_graph.cc
  #src/planners/polygon_stripmap_planner.cc
  #src/planners/polygon_stripmap_planner_exact.cc
  #src/planners/polygon_stripmap_planner_exact_preprocessed.cc
)
target_link_libraries(${PROJECT_NAME} ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})

#########
# TESTS #
#########
# catkin_add_gtest(test_plane_transformation
#   test/plane_transformation-test.cpp
# )
# target_link_libraries(test_plane_transformation ${PROJECT_NAME})
#
# catkin_add_gtest(test_polygon
#   test/polygon-test.cpp
# )
# target_link_libraries(test_polygon ${PROJECT_NAME})
#
# catkin_add_gtest(test_visibility_graph
#  test/visibility_graph-test.cpp
# )
# target_link_libraries(test_visibility_graph ${PROJECT_NAME})
#
# catkin_add_gtest(test_planners
#   test/planners-test.cpp
# )
# target_link_libraries(test_planners ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
