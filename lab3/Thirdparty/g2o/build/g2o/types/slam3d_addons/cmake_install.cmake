# Install script for directory: /home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/slam3d_addons

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libg2o_types_slam3d_addons.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libg2o_types_slam3d_addons.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libg2o_types_slam3d_addons.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/lib/libg2o_types_slam3d_addons.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libg2o_types_slam3d_addons.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libg2o_types_slam3d_addons.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libg2o_types_slam3d_addons.so"
         OLD_RPATH "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libg2o_types_slam3d_addons.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/g2o/types/slam3d_addons" TYPE FILE FILES
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/slam3d_addons/edge_plane.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/slam3d_addons/edge_se3_calib.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/slam3d_addons/edge_se3_euler.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/slam3d_addons/edge_se3_line.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/slam3d_addons/edge_se3_plane_calib.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/slam3d_addons/g2o_types_slam3d_addons_api.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/slam3d_addons/line3d.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/slam3d_addons/plane3d.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/slam3d_addons/types_slam3d_addons.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/slam3d_addons/vertex_line3d.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/slam3d_addons/vertex_plane.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/slam3d_addons/vertex_se3_euler.h"
    )
endif()

