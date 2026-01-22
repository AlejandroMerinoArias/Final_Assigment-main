# Library: tcpstreamreader
add_library(tcpstreamreader
  TCPStreamReader.cpp
)

# If the headers live next to the sources, expose this dir to consumers.
# Adjust/include an "include/" folder if you have one.
target_include_directories(tcpstreamreader
  PUBLIC
    ${LIBSOCKET_INCLUDE_DIR}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}/tcpimage>
)

# Link against libsocket (socket++) that you build via ExternalProject in the parent.
# Threads is a portable way to get -pthread where needed.
find_package(Threads REQUIRED)
target_link_libraries(tcpstreamreader
  PUBLIC
    socket++
    Threads::Threads
)

# Make sure the external libsocket is built before these libs try to link
add_dependencies(tcpstreamreader libsocket)

# Library: tcpimage (depends on tcpstreamreader)
add_library(tcpimage
  TCPImageServer.cpp
)

target_include_directories(tcpimage
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}/tcpimage>
)

target_link_libraries(tcpimage
  PUBLIC
    tcpstreamreader
)

add_dependencies(tcpimage libsocket)

# ---- Install (needed in ROS 2) ----------------------------------------------
include(GNUInstallDirs)

install(TARGETS
  tcpstreamreader
  tcpimage
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Install headers (adjust filenames/patterns to what you actually have)
# If you have an include/ folder, prefer installing that directory instead.
install(
  FILES
    TCPStreamReader.h
    TCPImageServer.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/tcpimage
)
