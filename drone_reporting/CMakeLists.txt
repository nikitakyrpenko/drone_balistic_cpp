add_library(drone_reporting STATIC
    src/ResultReporter.cpp
    src/FileResultReporter.cpp
    src/HttpResultReporter.cpp
)

target_include_directories(
    drone_reporting
    PUBLIC include ${CMAKE_CURRENT_SOURCE_DIR}/../external)

target_link_libraries(drone_reporting PUBLIC drone_model drone_transport)
target_compile_options(drone_reporting PRIVATE -Wall -Wextra -pedantic)

if(BUILD_TESTS AND NOT CMAKE_CROSSCOMPILING)
    # test sources are gitignored/untracked; only build the test target if they exist locally
    file(GLOB drone_reporting_test_sources CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp)
    if(drone_reporting_test_sources)
        add_executable(drone_reporting_tests ${drone_reporting_test_sources})
        target_include_directories(drone_reporting_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
        target_link_libraries(drone_reporting_tests PRIVATE GTest::gtest_main drone_reporting)
        target_compile_options(drone_reporting_tests PRIVATE -Wall -Wextra -pedantic)
        gtest_discover_tests(drone_reporting_tests DISCOVERY_MODE PRE_TEST PROPERTIES LABELS
                                                                                       "drone_reporting")
    endif()
endif()
