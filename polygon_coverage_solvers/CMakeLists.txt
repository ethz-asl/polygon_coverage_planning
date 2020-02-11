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
set(GKMA_INCLUDE_DIR ${CMAKE_BINARY_DIR}/gk_ma-prefix/src/gk_ma)
include(ExternalProject)
ExternalProject_Add(
  gk_ma
  URL https://polybox.ethz.ch/index.php/s/H4NXeaNPWo6VBrf/download
  DOWNLOAD_NAME gtsp_ma_source_codes.zip
  URL_MD5 765fad8e3746fa3dd9b81be0afb34d35
  PATCH_COMMAND \\
    patch GkMa/OurHeuristic/Algorithm.cs ${CMAKE_CURRENT_SOURCE_DIR}/patches/Algorithm.patch && \\
    patch NativeHelper/ClusterOptimisation.cpp ${CMAKE_CURRENT_SOURCE_DIR}/patches/ClusterOptimisationCpp.patch && \\
    patch GkMa/OurHeuristic/Types/Generation.cs ${CMAKE_CURRENT_SOURCE_DIR}/patches/Generation.patch && \\
    patch GkMa/OurHeuristic/GeneticAlgorithm.cs ${CMAKE_CURRENT_SOURCE_DIR}/patches/GeneticAlgorithm.patch && \\
    patch GkMa/Helper.cs ${CMAKE_CURRENT_SOURCE_DIR}/patches/Helper.patch && \\
    patch NativeHelper/ClusterOptimisation.h ${CMAKE_CURRENT_SOURCE_DIR}/patches/ClusterOptimisationH.patch && \\
    patch NativeHelper/ImprovementManager.h ${CMAKE_CURRENT_SOURCE_DIR}/patches/ImprovementManagerH.patch && \\
    patch NativeHelper/Insert.cpp ${CMAKE_CURRENT_SOURCE_DIR}/patches/InsertCpp.patch && \\
    patch NativeHelper/Insert.h ${CMAKE_CURRENT_SOURCE_DIR}/patches/InsertH.patch && \\
    patch NativeHelper/NativeHelper.cpp ${CMAKE_CURRENT_SOURCE_DIR}/patches/NativeHelperCpp.patch && \\
    patch NativeHelper/Swap.cpp ${CMAKE_CURRENT_SOURCE_DIR}/patches/SwapCpp.patch && \\
    patch NativeHelper/NativeHelper.h ${CMAKE_CURRENT_SOURCE_DIR}/patches/NativeHelperH.patch && \\
    patch GkMa/OurHeuristic/NativeHelper.cs ${CMAKE_CURRENT_SOURCE_DIR}/patches/NativeHelper.patch && \\
    patch GkMa/OurSolver.cs ${CMAKE_CURRENT_SOURCE_DIR}/patches/OurSolver.patch && \\
    patch GkMa/OurHeuristic/Types/Permutation.cs ${CMAKE_CURRENT_SOURCE_DIR}/patches/Permutation.patch && \\
    patch GkMa/Program.cs ${CMAKE_CURRENT_SOURCE_DIR}/patches/Program.patch && \\
    patch GkMa/Solver.cs ${CMAKE_CURRENT_SOURCE_DIR}/patches/Solver.patch && \\
    patch GkMa/Loader/Task.cs ${CMAKE_CURRENT_SOURCE_DIR}/patches/Task.patch && \\
    patch GkMa/OurHeuristic/Types/Tour.cs ${CMAKE_CURRENT_SOURCE_DIR}/patches/Tour.patch
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND \\
    cp ${PROJECT_SOURCE_DIR}/patches/MakefileCpp ./MakefileCpp && \\
    cp ${PROJECT_SOURCE_DIR}/patches/MakefileCs ./MakefileCs
  BUILD_COMMAND \\
    $(MAKE) -f MakefileCs BUILD_PATH="${CMAKE_LIBRARY_OUTPUT_DIRECTORY}" \\
    COMMAND $(MAKE) -f MakefileCpp BUILD_PATH="${CMAKE_LIBRARY_OUTPUT_DIRECTORY}"
  INSTALL_COMMAND \\
    COMMAND ${CMAKE_COMMAND} -E create_symlink /usr/lib/libmono-native.so ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/System.Native
)

# Download GTSP test instances.
include(ExternalProject)
ExternalProject_Add(
  gtsp_instances
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
