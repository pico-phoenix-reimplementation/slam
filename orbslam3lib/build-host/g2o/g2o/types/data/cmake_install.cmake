# Install script for directory: /home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/types/data

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
  set(CMAKE_INSTALL_SO_NO_EXE "0")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/iriska/picoreverse/slam/orbslam3lib/build-host/g2o/g2o/types/data/libg2o_types_data.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/g2o/types/data" TYPE FILE FILES
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/types/data/data_queue.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/types/data/g2o_types_data_api.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/types/data/laser_parameters.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/types/data/raw_laser.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/types/data/robot_data.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/types/data/robot_laser.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/types/data/types_data.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/types/data/vertex_ellipse.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/types/data/vertex_tag.h"
    )
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/iriska/picoreverse/slam/orbslam3lib/build-host/g2o/g2o/types/data/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
