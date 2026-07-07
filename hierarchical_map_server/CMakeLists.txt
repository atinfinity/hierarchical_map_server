cmake_minimum_required(VERSION 3.16)
project(hierarchical_map_server)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(tile_map_server REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Core logic (node-independent, target of unit tests)
add_library(${PROJECT_NAME}_core
  src/downsampler.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_core PUBLIC tile_map_server nav_msgs)
# Propagate the yaml-cpp referenced by tile_map_server_core (static lib) downstream
target_link_libraries(${PROJECT_NAME}_core PUBLIC yaml-cpp::yaml-cpp)

# Node body
add_executable(global_lowres_map_server
  src/global_lowres_map_server_node.cpp
  src/main.cpp
)
target_link_libraries(global_lowres_map_server ${PROJECT_NAME}_core)
ament_target_dependencies(global_lowres_map_server
  tile_map_server rclcpp rclcpp_lifecycle nav2_util nav_msgs
)

install(TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS global_lowres_map_server
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/make_lowres_map.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_downsampler test/test_downsampler.cpp)
  target_link_libraries(test_downsampler ${PROJECT_NAME}_core)

  ament_add_gtest(test_assemble test/test_assemble.cpp)
  target_link_libraries(test_assemble ${PROJECT_NAME}_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(tile_map_server nav_msgs)
ament_package()
