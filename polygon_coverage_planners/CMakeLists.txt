cmake_minimum_required(VERSION 3.16.3)
project(polygon_coverage_planners)

find_package(catkin REQUIRED COMPONENTS rosconsole roslib polygon_coverage_solvers polygon_coverage_geometry)

find_package(CGAL QUIET COMPONENTS Core)
include(${CGAL_USE_FILE})

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_BUILD_TYPE Release)

# TODO(rikba): Make catkin package.
find_package(PkgConfig)
pkg_check_modules(MONO mono-2 REQUIRED)
include_directories(${MONO_INCLUDE_DIRS})

catkin_package(
        INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
        LIBRARIES ${PROJECT_NAME}
)
include_directories(include ${catkin_INCLUDE_DIRS})

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}
  src/cost_functions/path_cost_functions.cc
  src/graphs/gtspp_product_graph.cc
  src/graphs/sweep_plan_graph.cc
  src/timing.cc
  src/planners/polygon_stripmap_planner.cc
  src/planners/polygon_stripmap_planner_exact.cc
  src/planners/polygon_stripmap_planner_exact_preprocessed.cc
)
target_link_libraries(${PROJECT_NAME} ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} ${catkin_LIBRARIES})

#########
# TESTS #
#########
catkin_add_gtest(test_planners
  test/planners-test.cpp
)
target_link_libraries(test_planners ${PROJECT_NAME} ${catkin_LIBRARIES})

##########
# EXPORT #
##########
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )

