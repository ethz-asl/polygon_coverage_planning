cmake_minimum_required(VERSION 3.0.2)

project(glkh)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel
endif()
add_compile_options(-Wall)
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -Wall -g")

set(COMMON_INCLUDES
    ${PROJECT_SOURCE_DIR}/SRC/INCLUDE
)

set(COMMON_SOURCES
    SRC/SolveGTSP.c
    SRC/ReadParameters.c
    SRC/PrintParameters.c
    SRC/Distance.c
    SRC/printff.c
    SRC/GetTime.c
    SRC/Makefile
    SRC/WriteTour.c
    SRC/ReadProblem.c
    SRC/PostOptimize.c
    SRC/SolveTSP.c
    SRC/fscanint.c
    SRC/Random.c
    SRC/ReadLine.c
    SRC/eprintf.c
    SRC/Statistics.c
    SRC/IsCandidate.c
    SRC/ReadPenalties.c
    SRC/Distance_SPECIAL.c
)

add_executable(LKH
    LKH-2.0.7/SRC/Heap.c
    LKH-2.0.7/SRC/GenerateCandidates.c
    LKH-2.0.7/SRC/SolveSubproblemBorderProblems.c
    LKH-2.0.7/SRC/GreedyTour.c
    LKH-2.0.7/SRC/SolveSFCSubproblems.c
    LKH-2.0.7/SRC/NormalizeNodeList.c
    LKH-2.0.7/SRC/ReadCandidates.c
    LKH-2.0.7/SRC/ChooseInitialTour.c
    LKH-2.0.7/SRC/IsCommonEdge.c
    LKH-2.0.7/SRC/Make2OptMove.c
    LKH-2.0.7/SRC/SolveRoheSubproblems.c
    LKH-2.0.7/SRC/Flip_SSL.c
    LKH-2.0.7/SRC/FindTour.c
    LKH-2.0.7/SRC/SolveCompressedSubproblem.c
    LKH-2.0.7/SRC/Ascent.c
    LKH-2.0.7/SRC/Between_SL.c
    LKH-2.0.7/SRC/RecordBestTour.c
    LKH-2.0.7/SRC/GeoConversion.c
    LKH-2.0.7/SRC/SolveSubproblem.c
    LKH-2.0.7/SRC/MakeKOptMove.c
    LKH-2.0.7/SRC/FreeStructures.c
    LKH-2.0.7/SRC/Activate.c
    LKH-2.0.7/SRC/ReadParameters.c
    LKH-2.0.7/SRC/Best3OptMove.c
    LKH-2.0.7/SRC/ResetCandidateSet.c
    LKH-2.0.7/SRC/PatchCycles.c
    LKH-2.0.7/SRC/SolveTourSegmentSubproblems.c
    LKH-2.0.7/SRC/MergeWithTour.c
    LKH-2.0.7/SRC/Minimum1TreeCost.c
    LKH-2.0.7/SRC/Gain23.c
    LKH-2.0.7/SRC/AllocateStructures.c
    LKH-2.0.7/SRC/PrintParameters.c
    LKH-2.0.7/SRC/Distance.c
    LKH-2.0.7/SRC/Make3OptMove.c
    LKH-2.0.7/SRC/IsPossibleCandidate.c
    LKH-2.0.7/SRC/CreateDelaunayCandidateSet.c
    LKH-2.0.7/SRC/SegmentSize.c
    LKH-2.0.7/SRC/AddTourCandidates.c
    LKH-2.0.7/SRC/CreateQuadrantCandidateSet.c
    LKH-2.0.7/SRC/BuildKDTree.c
    LKH-2.0.7/SRC/Flip_SL.c
    LKH-2.0.7/SRC/MergeTourWithBestTour.c
    LKH-2.0.7/SRC/Delaunay.c
    LKH-2.0.7/SRC/Genetic.c
    LKH-2.0.7/SRC/CandidateReport.c
    LKH-2.0.7/SRC/Sequence.c
    LKH-2.0.7/SRC/RecordBetterTour.c
    LKH-2.0.7/SRC/printff.c
    LKH-2.0.7/SRC/LKHmain.c
    LKH-2.0.7/SRC/AddExtraCandidates.c
    LKH-2.0.7/SRC/GetTime.c
    LKH-2.0.7/SRC/Between_SSL.c
    LKH-2.0.7/SRC/Hashing.c
    LKH-2.0.7/SRC/Best4OptMove.c
    LKH-2.0.7/SRC/WriteTour.c
    LKH-2.0.7/SRC/RestoreTour.c
    LKH-2.0.7/SRC/Forbidden.c
    LKH-2.0.7/SRC/SolveKCenterSubproblems.c
    LKH-2.0.7/SRC/C.c
    LKH-2.0.7/SRC/NormalizeSegmentList.c
    LKH-2.0.7/SRC/RemoveFirstActive.c
    LKH-2.0.7/SRC/MinimumSpanningTree.c
    LKH-2.0.7/SRC/BridgeGain.c
    LKH-2.0.7/SRC/Flip.c
    LKH-2.0.7/SRC/Best5OptMove.c
    LKH-2.0.7/SRC/Best2OptMove.c
    LKH-2.0.7/SRC/Excludable.c
    LKH-2.0.7/SRC/ReadProblem.c
    LKH-2.0.7/SRC/Connect.c
    LKH-2.0.7/SRC/StoreTour.c
    LKH-2.0.7/SRC/Between.c
    LKH-2.0.7/SRC/SolveDelaunaySubproblems.c
    LKH-2.0.7/SRC/AdjustCandidateSet.c
    LKH-2.0.7/SRC/LinKernighan.c
    LKH-2.0.7/SRC/WriteCandidates.c
    LKH-2.0.7/SRC/TrimCandidateSet.c
    LKH-2.0.7/SRC/Make4OptMove.c
    LKH-2.0.7/SRC/SFCTour.c
    LKH-2.0.7/SRC/SymmetrizeCandidateSet.c
    LKH-2.0.7/SRC/WritePenalties.c
    LKH-2.0.7/SRC/fscanint.c
    LKH-2.0.7/SRC/SolveKMeansSubproblems.c
    LKH-2.0.7/SRC/qsort.c
    LKH-2.0.7/SRC/Random.c
    LKH-2.0.7/SRC/FixedOrCommonCandidates.c
    LKH-2.0.7/SRC/OrderCandidateSet.c
    LKH-2.0.7/SRC/IsBackboneCandidate.c
    LKH-2.0.7/SRC/Make5OptMove.c
    LKH-2.0.7/SRC/SolveKarpSubproblems.c
    LKH-2.0.7/SRC/ERXT.c
    LKH-2.0.7/SRC/ReadLine.c
    LKH-2.0.7/SRC/AddCandidate.c
    LKH-2.0.7/SRC/eprintf.c
    LKH-2.0.7/SRC/Statistics.c
    LKH-2.0.7/SRC/BestKOptMove.c
    LKH-2.0.7/SRC/Exclude.c
    LKH-2.0.7/SRC/IsCandidate.c
    LKH-2.0.7/SRC/KSwapKick.c
    LKH-2.0.7/SRC/ReadPenalties.c
    LKH-2.0.7/SRC/CreateCandidateSet.c
    LKH-2.0.7/SRC/Distance_SPECIAL.c
)
target_include_directories(LKH PRIVATE
    ${PROJECT_SOURCE_DIR}/LKH-2.0.7/SRC/INCLUDE
)
target_link_libraries(LKH m)

set(GLKH_SOURCES SRC/GLKHmain.c)
add_executable(GLKH ${GLKH_SOURCES} ${COMMON_SOURCES})
target_include_directories(GLKH PRIVATE ${COMMON_INCLUDES})
target_link_libraries(GLKH m)

set(GLKH_EXP_SOURCES SRC/GLKH_EXPmain.c)
add_executable(GLKH_EXP ${GLKH_EXP_SOURCES} ${COMMON_SOURCES})
target_include_directories(GLKH_EXP PRIVATE ${COMMON_INCLUDES})
target_link_libraries(GLKH_EXP m)

set(GLKH_CHECK_SOURCES SRC/GLKH_CHECKmain.c)
add_executable(GLKH_CHECK ${GLKH_CHECK_SOURCES} ${COMMON_SOURCES})
target_include_directories(GLKH_CHECK PRIVATE ${COMMON_INCLUDES})
target_link_libraries(GLKH_CHECK m)

install(TARGETS LKH RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(TARGETS GLKH RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(TARGETS GLKH_EXP RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(TARGETS GLKH_CHECK RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
