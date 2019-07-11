cmake_minimum_required(VERSION 2.8.3)
project(polygon_coverage_msgs)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/msg_from_xml_rpc.cc
  src/conversion.cc
)
target_link_libraries(${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
