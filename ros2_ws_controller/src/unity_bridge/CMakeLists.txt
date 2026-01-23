cmake_minimum_required(VERSION 3.10)

if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW)
endif()

if(POLICY CMP0077)
  cmake_policy(SET CMP0077 NEW)
endif()

set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")

project(unity_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Dependencies ---------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# --- External projects ------------
include(ExternalProject)
include(GNUInstallDirs)  # gives CMAKE_INSTALL_{BINDIR,LIBDIR,…}

# Toggle to choose which Unity bundle to fetch (same logic as before)
option(TEST_FLAG "Choose alternative Unity package" ON)

if(TEST_FLAG)
  ExternalProject_Add(unitysim
    URL               https://www.dropbox.com/s/26525mbghkdqcd4/VNAV19_Unity.zip?dl=0
    DOWNLOAD_NAME     VNAV_Unity.zip
    CONFIGURE_COMMAND chmod +x ../unitysim/VNAV.x86_64
    BUILD_COMMAND     ""
    # In ROS 2, there is no CATKIN_DEVEL_PREFIX; install assets into share/
    INSTALL_COMMAND   ${CMAKE_COMMAND} -E make_directory ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/unitysim
                  &&  ${CMAKE_COMMAND} -E copy_directory ../unitysim ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/unitysim
  )
else()
  ExternalProject_Add(unitysim
    URL               https://www.dropbox.com/s/si54wh06udzpbd0/Linux_build_server_Data.zip?dl=0
    DOWNLOAD_NAME     Linux_build_server_Data.zip
    CONFIGURE_COMMAND chmod +x ../unitysim/Linux_build_server.x86_64
    BUILD_COMMAND     ""
    INSTALL_COMMAND   ${CMAKE_COMMAND} -E make_directory ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/unitysim
                  &&  ${CMAKE_COMMAND} -E copy_directory ../unitysim ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/unitysim
  )
endif()

ExternalProject_Get_Property(unitysim DOWNLOAD_DIR)
message(STATUS "UNITYSIM DOWNLOAD_DIR: ${DOWNLOAD_DIR}")

# libsocket external dependency (installed into current install prefix)
ExternalProject_Add(libsocket
  GIT_REPOSITORY https://github.com/dermesser/libsocket
  UPDATE_COMMAND ""
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
             -DCMAKE_BUILD_TYPE:STRING=Release
             -DBUILD_SHARED_LIBS=ON
             -DCMAKE_POLICY_DEFAULT_CMP0077=NEW
  BUILD_COMMAND $(MAKE)
  INSTALL_COMMAND $(MAKE) install
)

# Where ExternalProject will install headers and libs
set(LIBSOCKET_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include)
set(LIBSOCKET_LIBRARY_DIR ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})

# --- Includes / subdirs ------------------------------------------------------
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${LIBSOCKET_INCLUDE_DIR}
)

add_subdirectory(tcpimage)

# --- Executables --------------------------------------------------------------
add_executable(w_to_unity src/w_to_unity.cpp)
ament_target_dependencies(w_to_unity
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  mav_msgs
)

find_package(Threads REQUIRED)
target_link_libraries(w_to_unity socket++ Threads::Threads)
target_link_directories(w_to_unity PRIVATE ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})
add_dependencies(w_to_unity libsocket unitysim)

add_executable(unity_state src/unity_state.cpp)
ament_target_dependencies(unity_state
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)
target_link_libraries(unity_state tcpstreamreader)
target_link_directories(unity_state PRIVATE ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})
add_dependencies(unity_state libsocket)

add_executable(traj_publisher src/traj_publisher.cpp)
ament_target_dependencies(traj_publisher
  rclcpp
  std_msgs
  mav_msgs
  geometry_msgs
  trajectory_msgs
  tf2
  tf2_ros
)

# --- Install targets -------------------------------------
install(TARGETS
  w_to_unity
  unity_state
  traj_publisher
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install all launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# --- Export and package ------------------------------------------------------
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  Eigen3
)

ament_package()
