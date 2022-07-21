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

# Add mono to invoke gk_ma.
find_package(PkgConfig)
pkg_check_modules(MONO mono-2 REQUIRED)
include_directories(${MONO_INCLUDE_DIRS})

# Download gk_ma
set(GKMA_INCLUDE_DIR ${CMAKE_BINARY_DIR}/gk_ma-prefix/src/gk_ma)
include(ExternalProject)
ExternalProject_Add(
  gk_ma
  URL http://www.cs.nott.ac.uk/~pszdk/gtsp_ma_source_codes.zip
  URL https://polybox.ethz.ch/index.php/s/H4NXeaNPWo6VBrf/download
  DOWNLOAD_NAME gtsp_ma_source_codes.zip
  URL_MD5 765fad8e3746fa3dd9b81be0afb34d35
  PATCH_COMMAND
    COMMAND patch GkMa/OurHeuristic/Algorithm.cs ${CMAKE_CURRENT_SOURCE_DIR}/patches/Algorithm.patch
    COMMAND patch NativeHelper/ClusterOptimisation.cpp ${CMAKE_CURRENT_SOURCE_DIR}/patches/ClusterOptimisationCpp.patch
    COMMAND patch GkMa/OurHeuristic/Types/Generation.cs ${CMAKE_CURRENT_SOURCE_DIR}/patches/Generation.patch
    COMMAND patch GkMa/OurHeuristic/GeneticAlgorithm.cs ${CMAKE_CURRENT_SOURCE_DIR}/patches/GeneticAlgorithm.patch
    COMMAND patch GkMa/Helper.cs ${CMAKE_CURRENT_SOURCE_DIR}/patches/Helper.patch
    COMMAND patch NativeHelper/ClusterOptimisation.h ${CMAKE_CURRENT_SOURCE_DIR}/patches/ClusterOptimisationH.patch
    COMMAND patch NativeHelper/ImprovementManager.h ${CMAKE_CURRENT_SOURCE_DIR}/patches/ImprovementManagerH.patch
    COMMAND patch NativeHelper/Insert.cpp ${CMAKE_CURRENT_SOURCE_DIR}/patches/InsertCpp.patch
    COMMAND patch NativeHelper/Insert.h ${CMAKE_CURRENT_SOURCE_DIR}/patches/InsertH.patch
    COMMAND patch NativeHelper/NativeHelper.cpp ${CMAKE_CURRENT_SOURCE_DIR}/patches/NativeHelperCpp.patch
    COMMAND patch NativeHelper/Swap.cpp ${CMAKE_CURRENT_SOURCE_DIR}/patches/SwapCpp.patch
    COMMAND patch NativeHelper/NativeHelper.h ${CMAKE_CURRENT_SOURCE_DIR}/patches/NativeHelperH.patch
    COMMAND patch GkMa/OurHeuristic/NativeHelper.cs ${CMAKE_CURRENT_SOURCE_DIR}/patches/NativeHelper.patch
    COMMAND patch GkMa/OurSolver.cs ${CMAKE_CURRENT_SOURCE_DIR}/patches/OurSolver.patch
    COMMAND patch GkMa/OurHeuristic/Types/Permutation.cs ${CMAKE_CURRENT_SOURCE_DIR}/patches/Permutation.patch
    COMMAND patch GkMa/Program.cs ${CMAKE_CURRENT_SOURCE_DIR}/patches/Program.patch
    COMMAND patch GkMa/Solver.cs ${CMAKE_CURRENT_SOURCE_DIR}/patches/Solver.patch
    COMMAND patch GkMa/Loader/Task.cs ${CMAKE_CURRENT_SOURCE_DIR}/patches/Task.patch
    COMMAND patch GkMa/OurHeuristic/Types/Tour.cs ${CMAKE_CURRENT_SOURCE_DIR}/patches/Tour.patch
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND
    COMMAND cp ${PROJECT_SOURCE_DIR}/patches/MakefileCpp ./MakefileCpp
    COMMAND cp ${PROJECT_SOURCE_DIR}/patches/MakefileCs ./MakefileCs
  BUILD_COMMAND
    COMMAND $(MAKE) -f MakefileCs BUILD_PATH="${CMAKE_LIBRARY_OUTPUT_DIRECTORY}"
    COMMAND $(MAKE) -f MakefileCpp BUILD_PATH="${CMAKE_LIBRARY_OUTPUT_DIRECTORY}"
  INSTALL_COMMAND
    COMMAND ${CMAKE_COMMAND} -E create_symlink /usr/lib/libmono-native.so ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/System.Native
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
  src/gk_ma.cc
  src/combinatorics.cc
  src/boolean_lattice.cc
)
target_link_libraries(${PROJECT_NAME} ${MONO_LIBRARIES} ${catkin_LIBRARIES})

#########
# TESTS #
#########
catkin_add_gtest(test_combinatorics test/combinatorics-test.cpp)
target_link_libraries(test_combinatorics ${PROJECT_NAME} ${catkin_LIBRARIES})

catkin_add_gtest(test_gk_ma test/gk_ma-test.cpp)
target_link_libraries(test_gk_ma ${PROJECT_NAME} ${catkin_LIBRARIES})


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
