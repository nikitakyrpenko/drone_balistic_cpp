add_library(drone_state INTERFACE)

target_include_directories(drone_state INTERFACE include)

target_link_libraries(drone_state INTERFACE drone_model drone_utils)

if(BUILD_TESTS AND NOT CMAKE_CROSSCOMPILING)
    # test sources are gitignored/untracked; only build the test target if they exist locally
    file(GLOB drone_state_test_sources CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp)
    if(drone_state_test_sources)
        add_executable(drone_state_tests ${drone_state_test_sources})
        target_include_directories(drone_state_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
        target_link_libraries(drone_state_tests PRIVATE GTest::gtest_main drone_state)
        target_compile_options(drone_state_tests PRIVATE -Wall -Wextra -pedantic)
        gtest_discover_tests(drone_state_tests DISCOVERY_MODE PRE_TEST PROPERTIES LABELS
                                                                                  "drone_state")
    endif()
endif()
