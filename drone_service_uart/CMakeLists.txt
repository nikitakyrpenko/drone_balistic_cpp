add_library(
    drone_service_uart STATIC
    src/UartConfigLoader.cpp
    src/UartDronePhysics.cpp
    src/UartTargetProvider.cpp)

target_include_directories(drone_service_uart PUBLIC include)

target_link_libraries(drone_service_uart PUBLIC drone_core drone_transport)
target_compile_options(drone_service_uart PRIVATE -Wall -Wextra -pedantic)

if(BUILD_TESTS AND NOT CMAKE_CROSSCOMPILING)
    # test sources are gitignored/untracked; only build the test target if they exist locally
    file(GLOB drone_service_uart_test_sources CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp)
    if(drone_service_uart_test_sources)
        add_executable(drone_service_uart_tests ${drone_service_uart_test_sources})
        target_include_directories(drone_service_uart_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
        target_link_libraries(drone_service_uart_tests PRIVATE GTest::gtest_main drone_service_uart)
        target_compile_options(drone_service_uart_tests PRIVATE -Wall -Wextra -pedantic)
        gtest_discover_tests(drone_service_uart_tests DISCOVERY_MODE PRE_TEST PROPERTIES LABELS
                                                                                          "drone_service_uart")
    endif()
endif()
