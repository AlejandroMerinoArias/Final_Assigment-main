cmake_minimum_required(VERSION 3.8)
project(perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)

add_executable(lantern_detector src/lantern_detector_node.cpp)
ament_target_dependencies(lantern_detector
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_filters
)
target_link_libraries(lantern_detector ${OpenCV_LIBRARIES})

add_executable(lantern_detection_logger src/lantern_detection_logger_node.cpp)
ament_target_dependencies(lantern_detection_logger
  rclcpp
  geometry_msgs
)

install(TARGETS
  lantern_detector
  lantern_detection_logger
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES resource/${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
