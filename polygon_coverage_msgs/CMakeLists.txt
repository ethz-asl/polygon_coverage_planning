cmake_minimum_required(VERSION 3.16.3)
project(polygon_coverage_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs std_msgs trajectory_msgs xmlrpcpp polygon_coverage_geometry)

add_message_files(FILES PolygonWithHoles.msg PolygonWithHolesStamped.msg)
add_service_files(FILES PlannerService.srv PolygonService.srv)
generate_messages(DEPENDENCIES geometry_msgs std_msgs trajectory_msgs)

catkin_package(
        INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS message_runtime
)
include_directories(include ${catkin_INCLUDE_DIRS})

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}
  src/msg_from_xml_rpc.cc
  src/conversion.cc
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

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
