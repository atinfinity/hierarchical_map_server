cmake_minimum_required(VERSION 3.16)
project(tile_map_server)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

# コアロジック(ノード非依存、単体テスト対象)
add_library(${PROJECT_NAME}_core
  src/tileset.cpp
  src/pgm_loader.cpp
  src/tile_cache.cpp
  src/stitcher.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core PUBLIC yaml-cpp::yaml-cpp)
ament_target_dependencies(${PROJECT_NAME}_core PUBLIC nav_msgs)

# ノード本体
add_executable(tile_map_server_node
  src/tile_map_server_node.cpp
  src/main.cpp
)
target_link_libraries(tile_map_server_node ${PROJECT_NAME}_core)
ament_target_dependencies(tile_map_server_node
  rclcpp rclcpp_lifecycle nav2_util nav_msgs geometry_msgs tf2_ros
)

install(TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS tile_map_server_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config maps DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/split_map.py
  test/nav_across_boundary_test.py
  test/drive_across_boundary.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_tileset test/test_tileset.cpp)
  target_link_libraries(test_tileset ${PROJECT_NAME}_core)

  ament_add_gtest(test_stitcher test/test_stitcher.cpp)
  target_link_libraries(test_stitcher ${PROJECT_NAME}_core)

  ament_add_gtest(test_recenter test/test_recenter.cpp)
  target_link_libraries(test_recenter ${PROJECT_NAME}_core)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(nav_msgs yaml-cpp)
ament_package()
