cmake_minimum_required(VERSION 3.10)
project(fsm)

# --- C++ standard ---
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# --- Include directories ---
include_directories(include)

# --- Executable: mission_fsm_node ---
add_library(mission_fsm_node_lib
  src/mission_fsm_node.cpp
)

target_include_directories(mission_fsm_node_lib PUBLIC
  include
)

ament_target_dependencies(mission_fsm_node_lib
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  trajectory_msgs
  tf2
  tf2_ros
  visualization_msgs
)

add_executable(mission_fsm_node
  src/mission_fsm_main.cpp
)

target_link_libraries(mission_fsm_node
  mission_fsm_node_lib
)

ament_target_dependencies(mission_fsm_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  trajectory_msgs
  tf2
  tf2_ros
  visualization_msgs
)

# --- Install ---
install(TARGETS mission_fsm_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}
)

# --- Export ---
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  trajectory_msgs
  tf2
  tf2_ros
  visualization_msgs
)



if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(mission_fsm_node_test
    test/mission_fsm_node_test.cpp
  )
  
  target_link_libraries(mission_fsm_node_test
    mission_fsm_node_lib
  )
endif()


ament_package()
