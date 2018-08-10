cmake_minimum_required(VERSION 2.8.4)
project(mav_coverage_planning_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/nodes/2d/base_planner_2d.cc
  src/nodes/2d/shortest_path_2d.cc
  src/conversions/ros_interface.cc
)
target_link_libraries(${PROJECT_NAME})

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

##########
# EXPORT #
##########
cs_install()
cs_export()
