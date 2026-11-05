# Définit la version minimale de CMake requise
cmake_minimum_required(VERSION 3.22.1)
project(orbslam_service)

IF(NOT CMAKE_BUILD_TYPE)
        SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
if(NOT DEFINED ANDROID_ABI)
        set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
endif()

add_definitions(-D_UINT64_DEFINED -D_INT64_DEFINED)

# Check C++17 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
if(COMPILER_SUPPORTS_CXX17)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
        add_definitions(-DCOMPILEDWITHC17)
        message(STATUS "Using flag -std=c++17.")
else()
        message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif()

set(CMAKE_PROJECT_NAME androidorbslam3)

add_subdirectory(${CMAKE_SOURCE_DIR}/../../../external/orbslam3lib/app/src/main/cpp slam_lib)

set(CMAKE_PROJECT_NAME orbslam_service)

set(NATIVE_LIB_INCLUDE ${CMAKE_SOURCE_DIR}/../includes)
set(EXTERNAL_DIR ${CMAKE_SOURCE_DIR}/../../../external)

include_directories(${EXTERNAL_DIR}/include)
include_directories(${EXTERNAL_DIR}/include/glm)
include_directories(${EXTERNAL_DIR}/orbslam3lib/app/src/main/cpp/Thirdparty/eigen-3.4.0)
include_directories(${EXTERNAL_DIR}/orbslam3lib/app/src/main/cpp/include)
include_directories(${EXTERNAL_DIR}/orbslam3lib/app/src/main/cpp/include/CameraModels)
include_directories(${EXTERNAL_DIR}/orbslam3lib/app/src/main/cpp/Thirdparty/DBoW2/include)
include_directories(${EXTERNAL_DIR}/orbslam3lib/app/src/main/cpp/Thirdparty/sophus)
include_directories(${EXTERNAL_DIR}/orbslam3lib/app/src/main/cpp)
include_directories(${EXTERNAL_DIR}/orbslam3lib/app/src/main/cpp/Thirdparty/g2o)
include_directories(${g2o_BINARY_DIR})

add_library(LynxCapture SHARED IMPORTED)
set_target_properties(LynxCapture PROPERTIES IMPORTED_LOCATION ${EXTERNAL_DIR}/jniLibs/${ANDROID_ABI}/libLynxCapture.so)

add_library(openxr_loader SHARED IMPORTED)
set_target_properties(openxr_loader PROPERTIES IMPORTED_LOCATION ${EXTERNAL_DIR}/jniLibs/${ANDROID_ABI}/libopenxr_loader.so)

add_library(qxrsplitclient SHARED IMPORTED)
set_target_properties(qxrsplitclient PROPERTIES IMPORTED_LOCATION ${EXTERNAL_DIR}/jniLibs/${ANDROID_ABI}/libqxrsplitclient.so)

add_library(StereoKitC SHARED IMPORTED)
set_target_properties(StereoKitC PROPERTIES IMPORTED_LOCATION ${EXTERNAL_DIR}/jniLibs/${ANDROID_ABI}/libStereoKitC.so)


########################################################################
# Exporte ANativeActivity_onCreate(),
# Refer to: https://github.com/android-ndk/ndk/issues/381.
set(CMAKE_SHARED_LINKER_FLAGS
        "${CMAKE_SHARED_LINKER_FLAGS} -u ANativeActivity_onCreate")
########################################################################

file(GLOB NATIVE_ACTIVITY_LIB_SRC ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
add_library(orbslam_service SHARED ${NATIVE_ACTIVITY_LIB_SRC})

########################################################################
# Inclut les répertoires d'en-têtes nécessaires
#target_include_directories(orbslam_service PRIVATE
#        ${EXTERNAL_DIR}/include
#        ${NATIVE_LIB_INCLUDE}/)

########################################################################

# Hexagon SDK
# Set HEXAGON_SDK_ROOT and HEXAGON_CMAKE_ROOT
set(DSP_VERSION "v66")
set(HEXAGON_SDK_ROOT $ENV{HEXAGON_SDK_ROOT})
message("HEXAGON_SDK_ROOT ${HEXAGON_SDK_ROOT}")
set(HEXAGON_CMAKE_ROOT "${HEXAGON_SDK_ROOT}/build/cmake")
set(PREBUILT_LIB_DIR "android_aarch64")

# Add necessary includes for the project
set(common_incs
        ${CMAKE_CURRENT_BINARY_DIR}/
        ${HEXAGON_SDK_ROOT}/incs/
        ${HEXAGON_SDK_ROOT}/incs/stddef/
        ${HEXAGON_SDK_ROOT}/ipc/fastrpc/rpcmem/inc/
        ${HEXAGON_SDK_ROOT}/utils/examples
)

if(HEXAGON_SDK_ROOT)
        include(${HEXAGON_SDK_ROOT}/build/cmake/hexagon_fun.cmake)
else()
        include(${HEXAGON_CMAKE_ROOT}/hexagon_fun.cmake)
endif()

# Add include directories to the build
include_directories(${common_incs})

# Build interface files using user IDL
build_idl(${EXTERNAL_DIR}/orbslam3lib/app/src/main/cpp/inc/orbslam3.idl ${PROJECT_NAME})

# add lib dependencies
target_link_libraries(orbslam_service
        android
        EGL
        GLESv1_CM
        log
        m
        dl
        c
        androidorbslam3
        qxrsplitclient
        openxr_loader
        LynxCapture
        StereoKitC
)

