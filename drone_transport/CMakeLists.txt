find_library(GPIOD_LIBRARY gpiod REQUIRED)
find_path(GPIOD_INCLUDE_DIR gpiod.h REQUIRED)

add_library(drone_transport STATIC src/UartPort.cpp src/UartLink.cpp src/HttpRequest.cpp
                                   src/TcpPort.cpp src/TcpLink.cpp src/UdpPort.cpp)

target_include_directories(
    drone_transport
    PUBLIC include ${GPIOD_INCLUDE_DIR}
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../external)
target_link_libraries(drone_transport PUBLIC drone_model drone_utils mavlink ${GPIOD_LIBRARY})

if(BUILD_TESTS AND NOT CMAKE_CROSSCOMPILING)
    # test sources are gitignored/untracked; only build the test target if they exist locally
    file(GLOB drone_transport_test_sources CONFIGURE_DEPENDS
         ${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp)
    if(drone_transport_test_sources)
        add_executable(drone_transport_tests ${drone_transport_test_sources})
        target_include_directories(drone_transport_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
        target_link_libraries(drone_transport_tests PRIVATE GTest::gtest_main drone_transport)
        target_compile_options(drone_transport_tests PRIVATE -Wall -Wextra -pedantic)
        gtest_discover_tests(drone_transport_tests DISCOVERY_MODE PRE_TEST
                             PROPERTIES LABELS "drone_transport")
    endif()
endif()
