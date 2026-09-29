# Header-only for now: MavlinkLivenessService and its message packing live entirely in include/.
# Switch to STATIC with a source list once any of it moves into a .cpp.
add_library(drone_mavlink INTERFACE)

target_include_directories(drone_mavlink INTERFACE include)

target_link_libraries(drone_mavlink INTERFACE drone_model drone_utils drone_core drone_transport
                                              mavlink)

if(BUILD_TESTS AND NOT CMAKE_CROSSCOMPILING)
    # test sources are gitignored/untracked; only build the test target if they exist locally
    file(GLOB drone_mavlink_test_sources CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp)
    if(drone_mavlink_test_sources)
        add_executable(drone_mavlink_tests ${drone_mavlink_test_sources})
        target_include_directories(drone_mavlink_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
        target_link_libraries(drone_mavlink_tests PRIVATE GTest::gtest_main drone_mavlink)
        target_compile_options(drone_mavlink_tests PRIVATE -Wall -Wextra -pedantic)
        gtest_discover_tests(drone_mavlink_tests DISCOVERY_MODE PRE_TEST PROPERTIES LABELS
                                                                                    "drone_mavlink")
    endif()
endif()
