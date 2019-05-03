cmake_minimum_required(VERSION 2.8.3)
project(fm_rviz_polygon_tool)

add_definitions(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

######
# QT #
######
set(CMAKE_AUTOMOC ON)
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

############
# YAML-CPP #
############
# Link against system catkin yaml-cpp if installed.
find_package(PkgConfig)
find_package(yaml_cpp_catkin QUIET)
if(${yaml_cpp_catkin_FOUND})
    message(STATUS "Found yaml_cpp_catkin, using instead of system library.")
    set(YamlCpp_LIBRARIES ${yaml_cpp_catkin_LIBRARIES})
    set(YamlCpp_INCLUDE_DIRS ${yaml_cpp_catkin_INCLUDE_DIRS})
else()
    message(STATUS "No yaml_cpp_catkin, using yaml-cpp system library instead.")
    pkg_check_modules(YamlCpp REQUIRED yaml-cpp>=0.5)
endif()

#############
# LIBRARIES #
#############
# Header files as explained in https://answers.ros.org/question/215487/could-not-load-panel-in-rviz-pluginlibfactory-the-plugin-for-class/
set(HEADER_FILES
  include/fm_rviz_polygon_tool/polygon_tool.h

)
set(SRC_FILES
  src/polygon_tool.cc
)
cs_add_library(${PROJECT_NAME} ${SRC_FILES} ${HEADER_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${YamlCpp_LIBRARIES})

cs_install()
cs_export()
