cmake_minimum_required(VERSION 3.20)

project(drone_target_calc CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

enable_testing()

# tests temporarily disabled; re-enable with -DBUILD_TESTS=ON
option(BUILD_TESTS "Build unit tests" OFF)

if(BUILD_TESTS AND NOT CMAKE_CROSSCOMPILING)
    include(FetchContent)
    include(GoogleTest)
    FetchContent_Declare(
        googletest URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
                       DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    FetchContent_MakeAvailable(googletest)
endif()

add_subdirectory(external)

add_subdirectory(drone_model)
add_subdirectory(drone_utils)
add_subdirectory(drone_state)
add_subdirectory(drone_transport)
add_subdirectory(drone_mavlink)
add_subdirectory(drone_core)
add_subdirectory(drone_service_json)
add_subdirectory(drone_service_uart)
add_subdirectory(drone_mission_factory)
add_subdirectory(drone_reporting)

add_executable(drone_target_cli src/Main.cpp)
target_link_libraries(
    drone_target_cli
    PRIVATE drone_model
            drone_state
            drone_core
            drone_service_uart
            drone_transport
            drone_mission_factory
            drone_reporting
            drone_mavlink
            mavlink)
target_compile_options(drone_target_cli PRIVATE -Wall -Wextra -pedantic)
