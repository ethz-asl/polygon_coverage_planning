cmake_minimum_required(VERSION 3.16.3)
project(polygon_coverage_solvers)

find_package(catkin REQUIRED COMPONENTS rosconsole roslib)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

catkin_package(
        INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
        LIBRARIES ${PROJECT_NAME}
)
include_directories(include ${catkin_INCLUDE_DIRS})

# Download glkh
include(ExternalProject)
ExternalProject_Add(
  glkh
  URL http://webhotel4.ruc.dk/~keld/research/GLKH/GLKH-1.0.tgz
  DOWNLOAD_NAME glkh_source_codes.tgz
  URL_MD5 9b0ba92053dac798f550c5c8a9524120
  BINARY_DIR ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}
  PATCH_COMMAND
      COMMAND patch --forward ./SRC/SolveTSP.c < ${PROJECT_SOURCE_DIR}/patches/SolveTSP.patch
      COMMAND cp ${PROJECT_SOURCE_DIR}/patches/CMakeLists.txt ./
  INSTALL_COMMAND ""
)

# Download GTSP test instances.
include(ExternalProject)
ExternalProject_Add(
  gtsp_instances
  URL http://www.cs.nott.ac.uk/~dxk/gtsplib/InstancesBinary.zip
  URL https://polybox.ethz.ch/index.php/s/51iqurpOOQ5cVaJ/download
  DOWNLOAD_NAME InstancesBinary.zip
  URL_MD5 255831bd47de71df8419a54741f0a6be
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
)

#############
# LIBRARIES #
#############
add_library(${PROJECT_NAME}
  src/glkh.cc
  src/combinatorics.cc
  src/boolean_lattice.cc
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#########
# TESTS #
#########
catkin_add_gtest(test_combinatorics
  test/combinatorics-test.cpp
)
target_link_libraries(test_combinatorics ${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_add_gtest(test_glkh test/glkh-test.cpp)
target_link_libraries(test_glkh ${PROJECT_NAME} ${catkin_LIBRARIES})


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
