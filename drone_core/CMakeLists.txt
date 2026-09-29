add_library(
    drone_core STATIC
    src/MissionProccesor.cpp
    src/FirepointProvider.cpp
    src/BallisticSolutionEvaluator.cpp
    src/AnalyticalBallisticSolver.cpp
    src/TableBallisticSolver.cpp
    src/BallisticTableLoader.cpp)

target_include_directories(drone_core PUBLIC include)

target_link_libraries(drone_core PUBLIC drone_model drone_state drone_utils)
target_compile_options(drone_core PRIVATE -Wall -Wextra -pedantic)

if(BUILD_TESTS AND NOT CMAKE_CROSSCOMPILING)
    # test sources are gitignored/untracked; only build the test target if they exist locally
    file(GLOB drone_core_test_sources CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp)
    if(drone_core_test_sources)
        add_executable(drone_core_tests ${drone_core_test_sources})
        target_include_directories(drone_core_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
        target_link_libraries(drone_core_tests PRIVATE GTest::gtest_main drone_core)
        target_compile_options(drone_core_tests PRIVATE -Wall -Wextra -pedantic)
        gtest_discover_tests(drone_core_tests DISCOVERY_MODE PRE_TEST PROPERTIES LABELS
                                                                                  "drone_core")
    endif()
endif()
