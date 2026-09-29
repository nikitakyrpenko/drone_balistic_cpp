add_library(drone_utils INTERFACE)

target_include_directories(drone_utils INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(drone_utils INTERFACE drone_model)
