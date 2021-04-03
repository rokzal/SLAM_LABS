# Install script for directory: /home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/apps/g2o_simulator

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
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libg2o_simulator.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libg2o_simulator.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libg2o_simulator.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/lib/libg2o_simulator.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libg2o_simulator.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libg2o_simulator.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libg2o_simulator.so"
         OLD_RPATH "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libg2o_simulator.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/g2o_simulator2d" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/g2o_simulator2d")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/g2o_simulator2d"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/bin/g2o_simulator2d")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/g2o_simulator2d" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/g2o_simulator2d")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/g2o_simulator2d"
         OLD_RPATH "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/g2o_simulator2d")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/g2o_simulator3d" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/g2o_simulator3d")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/g2o_simulator3d"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/bin/g2o_simulator3d")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/g2o_simulator3d" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/g2o_simulator3d")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/g2o_simulator3d"
         OLD_RPATH "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/g2o_simulator3d")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/g2o_anonymize_observations" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/g2o_anonymize_observations")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/g2o_anonymize_observations"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/bin/g2o_anonymize_observations")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/g2o_anonymize_observations" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/g2o_anonymize_observations")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/g2o_anonymize_observations"
         OLD_RPATH "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/g2o_anonymize_observations")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/g2o/apps/g2o_simulator" TYPE FILE FILES
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/apps/g2o_simulator/g2o_simulator_api.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/apps/g2o_simulator/pointsensorparameters.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/apps/g2o_simulator/sensor_line3d.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/apps/g2o_simulator/sensor_odometry.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/apps/g2o_simulator/sensor_odometry2d.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/apps/g2o_simulator/sensor_odometry3d.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/apps/g2o_simulator/sensor_pointxy.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/apps/g2o_simulator/sensor_pointxy_bearing.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/apps/g2o_simulator/sensor_pointxy_offset.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/apps/g2o_simulator/sensor_pointxyz.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/apps/g2o_simulator/sensor_pointxyz_depth.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/apps/g2o_simulator/sensor_pointxyz_disparity.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/apps/g2o_simulator/sensor_pose2d.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/apps/g2o_simulator/sensor_pose3d.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/apps/g2o_simulator/sensor_pose3d_offset.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/apps/g2o_simulator/sensor_se3_prior.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/apps/g2o_simulator/sensor_segment2d.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/apps/g2o_simulator/sensor_segment2d_line.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/apps/g2o_simulator/sensor_segment2d_pointline.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/apps/g2o_simulator/simulator.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/apps/g2o_simulator/simulator2d.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/apps/g2o_simulator/simulator2d_base.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/apps/g2o_simulator/simulator3d.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/apps/g2o_simulator/simulator3d_base.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/apps/g2o_simulator/simutils.h"
    )
endif()

