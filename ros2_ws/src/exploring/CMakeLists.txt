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
find_package(std_msgs REQUIRED)

include_directories(include)


# 1. Build the library (logic only)
add_library(exploration_manager_lib src/exploration_manager.cpp)
ament_target_dependencies(exploration_manager_lib
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  std_msgs
)

# 2. Build the executable (wraps the lib)
add_executable(exploration_manager src/exploration_manager_main.cpp)
target_link_libraries(exploration_manager exploration_manager_lib)
ament_target_dependencies(exploration_manager
  rclcpp
)

install(TARGETS exploration_manager exploration_manager_lib
  DESTINATION lib/${PROJECT_NAME}
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
  ament_add_gtest(exploration_manager_test test/exploration_manager_test.cpp)
  target_link_libraries(exploration_manager_test exploration_manager_lib)
endif()

ament_package()
