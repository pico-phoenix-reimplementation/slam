# Install script for directory: /home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/stuff

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/iriska/picoreverse/slam/orbslam3lib/build-host/g2o/g2o/stuff/libg2o_stuff.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/g2o/stuff" TYPE FILE FILES
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/stuff/color_macros.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/stuff/command_args.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/stuff/filesys_tools.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/stuff/g2o_stuff_api.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/stuff/logger.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/stuff/logger_format.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/stuff/macros.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/stuff/misc.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/stuff/opengl_primitives.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/stuff/opengl_wrapper.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/stuff/property.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/stuff/sampler.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/stuff/sparse_helper.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/stuff/string_tools.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/stuff/tictoc.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/stuff/timeutil.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/stuff/tuple_tools.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/stuff/unscented.h"
    )
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/iriska/picoreverse/slam/orbslam3lib/build-host/g2o/g2o/stuff/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
