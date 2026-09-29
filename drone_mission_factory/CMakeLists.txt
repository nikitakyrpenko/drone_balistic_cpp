add_library(drone_mission_factory STATIC src/MissionFactory.cpp)

target_include_directories(drone_mission_factory PUBLIC include)

target_link_libraries(drone_mission_factory PUBLIC drone_core drone_service_json drone_service_uart)
target_compile_options(drone_mission_factory PRIVATE -Wall -Wextra -pedantic)

if(BUILD_TESTS AND NOT CMAKE_CROSSCOMPILING)
    # test sources are gitignored/untracked; only build the test target if they exist locally
    file(GLOB drone_mission_factory_test_sources CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp)
    if(drone_mission_factory_test_sources)
        add_executable(drone_mission_factory_tests ${drone_mission_factory_test_sources})
        target_include_directories(drone_mission_factory_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
        target_link_libraries(drone_mission_factory_tests PRIVATE GTest::gtest_main drone_mission_factory)
        target_compile_options(drone_mission_factory_tests PRIVATE -Wall -Wextra -pedantic)
        gtest_discover_tests(drone_mission_factory_tests DISCOVERY_MODE PRE_TEST PROPERTIES LABELS
                                                                                             "drone_mission_factory")
    endif()
endif()
