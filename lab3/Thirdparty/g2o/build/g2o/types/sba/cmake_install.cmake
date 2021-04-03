# Install script for directory: /home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/sba

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
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libg2o_types_sba.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libg2o_types_sba.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libg2o_types_sba.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/lib/libg2o_types_sba.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libg2o_types_sba.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libg2o_types_sba.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libg2o_types_sba.so"
         OLD_RPATH "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libg2o_types_sba.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/g2o/types/sba" TYPE FILE FILES
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/sba/edge_project_p2mc.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/sba/edge_project_p2sc.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/sba/edge_project_psi2uv.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/sba/edge_project_stereo_xyz.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/sba/edge_project_stereo_xyz_onlypose.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/sba/edge_project_xyz.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/sba/edge_project_xyz2uv.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/sba/edge_project_xyz2uvu.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/sba/edge_project_xyz_onlypose.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/sba/edge_sba_cam.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/sba/edge_sba_scale.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/sba/edge_se3_expmap.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/sba/g2o_types_sba_api.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/sba/parameter_cameraparameters.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/sba/sba_utils.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/sba/sbacam.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/sba/types_sba.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/sba/types_six_dof_expmap.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/sba/vertex_cam.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/sba/vertex_intrinsics.h"
    "/home/ubaldo/SLAM/SLAM_LABS/lab3/Thirdparty/g2o/g2o/types/sba/vertex_se3_expmap.h"
    )
endif()

