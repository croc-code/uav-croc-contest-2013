cmake_minimum_required(VERSION 2.8)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo REQUIRED)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(/usr/include/gazebo-1.9/gazebo)
include_directories(/usr/include/gazebo-1.9)
include_directories(/usr/include/sdformat-1.4)

include(cmake/rospack.cmake)


get_rospack_flags(roscpp)
include_directories(${roscpp_INCLUDE_DIRS})
link_directories(${roscpp_LIBRARY_DIRS})
set(CMAKE_INSTALL_RPATH "${roscpp_LIBRARY_DIRS}")
add_library(ros_lidar_plugin SHARED ros_lidar_plugin.cc)
set_target_properties(ros_lidar_plugin PROPERTIES COMPILE_FLAGS "${roscpp_CFLAGS_OTHER}")
set_target_properties(ros_lidar_plugin PROPERTIES LINK_FLAGS "${roscpp_LDFLAGS_OTHER}")
target_link_libraries(ros_lidar_plugin ${roscpp_LIBRARIES})
install (TARGETS ros_lidar_plugin DESTINATION ~/local/lib/gazebo_plugins/)


