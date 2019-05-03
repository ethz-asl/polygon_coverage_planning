cmake_minimum_required(VERSION 2.8.4)
project(polygon_coverage_solvers)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# Add mono to invoke gk_ma.
find_package(PkgConfig)
pkg_check_modules(MONO mono-2 REQUIRED)
include_directories(${MONO_INCLUDE_DIRS})

# Download gk_ma
include(ExternalProject)
ExternalProject_Add(
  gk_ma
  URL http://www.cs.nott.ac.uk/~dxk/gtsp_ma_source_codes.zip
  URL_MD5 765fad8e3746fa3dd9b81be0afb34d35
  PATCH_COMMAND \\
    patch NativeHelper/ClusterOptimisation.h ${CMAKE_CURRENT_SOURCE_DIR}/patches/cluster_optimisation.patch && \\
    patch NativeHelper/ImprovementManager.h ${CMAKE_CURRENT_SOURCE_DIR}/patches/improvement_manager.patch
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
)

# Compile gk_ma.
set(GKMA_INCLUDE_DIR include/polygon_coverage_solvers/gk_ma/gk_ma)
set(GKMA_FULL_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/${GKMA_INCLUDE_DIR})
set(GKMA gkma)
add_custom_target(${GKMA} ALL
  COMMAND $(MAKE) BUILD_PATH="${CMAKE_LIBRARY_OUTPUT_DIRECTORY}"
  WORKING_DIRECTORY ${GKMA_FULL_INCLUDE_DIR})

# Download GTSP test instances.
include(ExternalProject)
ExternalProject_Add(
  gtsp_instances
  URL http://www.cs.nott.ac.uk/~dxk/gtsplib/InstancesBinary.zip
  URL_MD5 255831bd47de71df8419a54741f0a6be
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
)

#############
# LIBRARIES #
#############
# GK_MA c++ library
set(NATIVE_HELPER native_helper)
cs_add_library(${NATIVE_HELPER}
  src/native_helper/ClusterOptimisation.cpp
  src/native_helper/ImprovementManager.cpp
  src/native_helper/Insert.cpp
  src/native_helper/KOpt.cpp
  src/native_helper/NativeHelper.cpp
  src/native_helper/Swap.cpp
)

cs_add_library(${PROJECT_NAME}
  src/gk_ma.cc
  src/combinatorics.cc
  src/boolean_lattice.cc
)
target_link_libraries(${PROJECT_NAME} ${MONO_LIBRARIES})

#########
# TESTS #
#########
catkin_add_gtest(test_combinatorics
  test/combinatorics-test.cpp
)
target_link_libraries(test_combinatorics
                      ${PROJECT_NAME})

catkin_add_gtest(test_gk_ma
  test/gk_ma-test.cpp
)
target_link_libraries(test_gk_ma
                      ${PROJECT_NAME})


##########
# EXPORT #
##########
cs_install()
cs_export()
