cmake_minimum_required(VERSION 3.8)
project(indoor_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =====================
# Dependencies
# =====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# =====================
# Library
# =====================
add_library(indoor_navigation
  src/explore.cpp
#  src/frontier_search.cpp
)

target_include_directories(indoor_navigation
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(indoor_navigation
  rclcpp
  nav_msgs
  map_msgs
  geometry_msgs
  nav2_costmap_2d
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# =====================
# Executable
# =====================
add_executable(explorer src/explore.cpp)

target_link_libraries(explorer
  indoor_navigation
)

ament_target_dependencies(explorer
  rclcpp
)

# =====================
# Install
# =====================
install(
  TARGETS indoor_navigation explorer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# =====================
# Tests
# =====================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()