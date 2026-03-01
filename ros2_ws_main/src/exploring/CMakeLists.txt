cmake_minimum_required(VERSION 3.8)
project(exploring)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetExplorationGoal.srv"
  DEPENDENCIES geometry_msgs
)

include_directories(include)
include_directories(${OCTOMAP_INCLUDE_DIRS})


# 1. Build the library (logic only)
add_library(exploration_manager_lib src/exploration_manager.cpp)
ament_target_dependencies(exploration_manager_lib
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  octomap_msgs
  octomap
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(exploration_manager_lib "${cpp_typesupport_target}")
target_link_libraries(exploration_manager_lib ${OCTOMAP_LIBRARIES})

# 2. Build the executable (wraps the lib)
add_executable(exploration_manager src/exploration_manager_main.cpp)
target_link_libraries(exploration_manager exploration_manager_lib)
ament_target_dependencies(exploration_manager
  rclcpp
)

install(TARGETS exploration_manager_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS exploration_manager
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install RViz config
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Install Python scripts
install(PROGRAMS
  scripts/mock_map_publisher.py
  scripts/mock_drone_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Add GoogleTest
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
