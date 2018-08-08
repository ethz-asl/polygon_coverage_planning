cmake_minimum_required(VERSION 2.8.4)
project(mav_coverage_graph_solvers)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# Add mono to invoke gk_ma.
find_package(PkgConfig)
pkg_check_modules(MONO mono-2 REQUIRED)
include_directories(${MONO_INCLUDE_DIRS})

# Compile gk_ma.
set(GKMA_INCLUDE_DIR include/mav_coverage_graph_solvers/gk_ma/gk_ma)
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

##########
# EXPORT #
##########
cs_install()
cs_export()
