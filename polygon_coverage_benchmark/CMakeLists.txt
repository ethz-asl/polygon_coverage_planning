cmake_minimum_required(VERSION 3.16.3)
project(polygon_coverage_benchmark)

find_package(catkin REQUIRED COMPONENTS
            rosconsole
            roslib
            polygon_coverage_msgs
            polygon_coverage_geometry
            polygon_coverage_planners
            polygon_coverage_solvers
            )


#set(CMAKE_CXX_STANDARD 17)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_BUILD_TYPE Release)

find_package(CGAL QUIET COMPONENTS Core)
include(${CGAL_USE_FILE})

find_package(yaml-cpp REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(glog REQUIRED libglog)

catkin_package()
include_directories(include ${CGAL_INCLUDE_DIRS}
                            ${catkin_INCLUDE_DIRS}
                            ${YAML_INCLUDE_DIRS}
                            ${glog_INCLUDE_DIRS})

catkin_add_gtest(test_benchmark test/benchmark-test.cpp)
target_link_libraries(test_benchmark ${CGAL_LIBRARIES} ${YAML_CPP_LIBRARIES} ${glog_LIBRARIES} ${catkin_LIBRARIES} )

catkin_python_setup()
