cmake_minimum_required(VERSION 2.8.4)
project(mav_coverage_planning_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(CGAL QUIET COMPONENTS Core)
include(${CGAL_USE_FILE})

add_definitions(-std=c++11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

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

##########
# EXPORT #
##########
cs_install()
cs_export()
