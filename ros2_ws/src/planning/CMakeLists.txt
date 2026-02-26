cmake_minimum_required(VERSION 3.8)
project(planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${OCTOMAP_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

# Global planner executable
add_executable(global_planner_node
  src/global_planner_node.cpp
)

target_include_directories(global_planner_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OCTOMAP_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(global_planner_node
  ${OCTOMAP_LIBRARIES}
)

ament_target_dependencies(global_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  octomap
  octomap_msgs
  Eigen3
)

# Install executable
install(TARGETS
  global_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
