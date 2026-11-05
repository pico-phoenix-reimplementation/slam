# Linux build of the Lynx ORB-SLAM3 library (PICO 4 on mainline Linux, or a
# development host).
#
#   cmake -S linux -B build -DLYNX_BACKEND=ref       # CPU reference of the DSP interface
#   cmake -S linux -B build -DLYNX_BACKEND=hexagonrpc \
#         -DHEXAGONRPC_SRC=<hexagonrpc checkout>  # feature extraction on the CDSP
#
# hexagonrpc: a hand-written FastRPC client on top of hexagonrpcd's FastRPC
# fd; no Hexagon SDK and no libcdsprpc needed on the headset. The DSP skel is
# built on the host from ../dsp (dsp/build.sh).

cmake_minimum_required(VERSION 3.16)
project(lynx_orbslam3 C CXX)

set(LYNX_BACKEND "ref" CACHE STRING "Feature extraction backend: ref (CPU) or hexagonrpc (CDSP)")
set_property(CACHE LYNX_BACKEND PROPERTY STRINGS ref hexagonrpc)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    # SM8250 big cores (Lynx used the same flags on the XR2).
    set(ARCH_FLAGS "-mcpu=cortex-a77+crc+simd")
else()
    set(ARCH_FLAGS "-march=native")
endif()
set(CMAKE_C_FLAGS_RELEASE "-O3 -ffast-math -ftree-vectorize -funroll-loops ${ARCH_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -ffast-math -ftree-vectorize -funroll-loops ${ARCH_FLAGS}")

add_definitions(-DCOMPILEDWITHC11 -DCOMPILEDWITHC17)

# musl (Alpine/postmarketOS on the headset) gives threads a 128 KiB stack;
# ORB-SLAM3's frame thread alone puts ~300 KB of Frame objects on its stack.
# musl sizes the default thread stack from PT_GNU_STACK, so ask for 8 MiB
# (glibc's default; harmless there).
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,stack-size=8388608")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-z,stack-size=8388608")

set(SRC ${CMAKE_CURRENT_SOURCE_DIR}/../app/src/main/cpp)

find_package(OpenCV REQUIRED COMPONENTS core flann geometry imgproc photo features imgcodecs video videoio calib highgui)
find_package(Threads REQUIRED)

# g2o (static, as in the Android build)
set(G2O_BUILD_APPS OFF CACHE BOOL "" FORCE)
set(G2O_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(G2O_USE_OPENGL OFF CACHE BOOL "" FORCE)
set(G2O_USE_CSPARSE OFF CACHE BOOL "" FORCE)
set(G2O_USE_CHOLMOD OFF CACHE BOOL "" FORCE)
set(G2O_USE_LOGGING OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF)
set(EIGEN3_INCLUDE_DIR ${SRC}/Thirdparty/eigen-3.4.0)
list(APPEND CMAKE_PREFIX_PATH ${SRC}/Thirdparty/eigen-3.4.0)
add_subdirectory(${SRC}/Thirdparty/g2o ${CMAKE_BINARY_DIR}/g2o EXCLUDE_FROM_ALL)

file(GLOB DBOW2_SOURCES ${SRC}/Thirdparty/DBoW2/src/*.cpp)

add_library(orbslam3 SHARED
    ${SRC}/src/System.cc
    ${SRC}/src/Tracking.cc
    ${SRC}/src/LocalMapping.cc
    ${SRC}/src/LoopClosing.cc
    ${SRC}/src/ORBextractor.cc
    ${SRC}/src/ORBmatcher.cc
    ${SRC}/src/Converter.cc
    ${SRC}/src/MapPoint.cc
    ${SRC}/src/KeyFrame.cc
    ${SRC}/src/Atlas.cc
    ${SRC}/src/Map.cc
    ${SRC}/src/Optimizer.cc
    ${SRC}/src/Frame.cc
    ${SRC}/src/KeyFrameDatabase.cc
    ${SRC}/src/Sim3Solver.cc
    ${SRC}/src/ImuTypes.cc
    ${SRC}/src/G2oTypes.cc
    ${SRC}/src/CameraModels/Pinhole.cpp
    ${SRC}/src/CameraModels/KannalaBrandt8.cpp
    ${SRC}/src/OptimizableTypes.cpp
    ${SRC}/src/MLPnPsolver.cpp
    ${SRC}/src/GeometricTools.cc
    ${SRC}/src/TwoViewReconstruction.cc
    ${SRC}/src/Config.cc
    ${SRC}/src/Settings.cc
    ${SRC}/src/LynxHardwareAcceleration/LynxHardwareAccelerator.cpp
    ${SRC}/src/LynxHardwareAcceleration/FrameAHB.cc
    ${DBOW2_SOURCES}
)

target_include_directories(orbslam3 PUBLIC
    ${SRC}
    ${SRC}/include
    ${SRC}/include/CameraModels
    ${SRC}/Thirdparty/Sophus
    ${SRC}/Thirdparty/eigen-3.4.0
    ${SRC}/Thirdparty/DBoW2/include
    ${SRC}/Thirdparty/DBoW2/include/DBoW2
    ${SRC}/Thirdparty/g2o
    ${CMAKE_BINARY_DIR}/g2o
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(orbslam3 PUBLIC
    ${OpenCV_LIBS}
    g2o::core g2o::stuff g2o::types_sba g2o::types_sim3 g2o::solver_eigen
    Threads::Threads
)

if(LYNX_BACKEND STREQUAL "ref")
    target_sources(orbslam3 PRIVATE ${SRC}/src/LynxHardwareAcceleration/orbslam3_ref.cpp)
    target_include_directories(orbslam3 PUBLIC ${SRC}/inc/ref)

    # One frame through the CPU reference, for comparison with the DSP (dsp/sim).
    add_executable(orb_dump ${CMAKE_CURRENT_SOURCE_DIR}/orb_dump.cc ${SRC}/src/LynxHardwareAcceleration/orbslam3_ref.cpp)
    target_include_directories(orb_dump PRIVATE ${SRC}/inc/ref ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(orb_dump ${OpenCV_LIBS} Threads::Threads)
elseif(LYNX_BACKEND STREQUAL "hexagonrpc")
    # CDSP through the mainline fastrpc driver, without libcdsprpc or qaic:
    # hexagonrpc_client.c marshals the calls by hand (inc/orbslam3_rpc.h, the
    # skel is dsp/src/orbslam3_skel_hrpc.c) on the FastRPC fd handed over by
    # hexagonrpcd (linux-msm/hexagonrpc, tools/fastrpc on the PICO 4 side). The
    # program must be started by "hexagonrpcd ... -p <program>".
    set(HEXAGONRPC_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/hexagonrpc CACHE PATH "linux-msm/hexagonrpc v0.4.0 checkout")
    if(NOT EXISTS ${HEXAGONRPC_SRC}/libhexagonrpc/fastrpc.c)
        message(FATAL_ERROR "LYNX_BACKEND=hexagonrpc needs -DHEXAGONRPC_SRC=<linux-msm/hexagonrpc v0.4.0 checkout>")
    endif()
    target_sources(orbslam3 PRIVATE
        ${SRC}/src/LynxHardwareAcceleration/hexagonrpc_client.c
        ${HEXAGONRPC_SRC}/libhexagonrpc/fastrpc.c
        ${HEXAGONRPC_SRC}/libhexagonrpc/interfaces.c)
    target_include_directories(orbslam3 PUBLIC ${SRC}/inc/hexagonrpc ${SRC}/inc)
    target_include_directories(orbslam3 PRIVATE ${HEXAGONRPC_SRC}/include)
    # The CPU reference as fallback when the CDSP cannot be used (ORBSLAM3_FORCE_CPU=1 forces it).
    target_sources(orbslam3 PRIVATE ${SRC}/src/LynxHardwareAcceleration/orbslam3_ref.cpp)
    set_source_files_properties(${SRC}/src/LynxHardwareAcceleration/orbslam3_ref.cpp PROPERTIES COMPILE_DEFINITIONS ORBSLAM3_REF_FALLBACK)
    target_compile_definitions(orbslam3 PRIVATE LYNX_CPU_FALLBACK)

    # One frame through the CDSP (compare with orb_dump of a ref build).
    # Needs only the client, not the SLAM library: the first thing to try on the headset.
    add_executable(orb_dump_cdsp ${CMAKE_CURRENT_SOURCE_DIR}/orb_dump.cc
        ${SRC}/src/LynxHardwareAcceleration/hexagonrpc_client.c
        ${HEXAGONRPC_SRC}/libhexagonrpc/fastrpc.c
        ${HEXAGONRPC_SRC}/libhexagonrpc/interfaces.c)
    target_include_directories(orb_dump_cdsp PRIVATE ${SRC}/inc/hexagonrpc ${SRC}/inc ${HEXAGONRPC_SRC}/include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(orb_dump_cdsp ${OpenCV_LIBS})
else()
    message(FATAL_ERROR "Unknown LYNX_BACKEND ${LYNX_BACKEND}")
endif()

# Stereo runner: feeds side-by-side frames from a directory and writes the trajectory.
add_executable(lynx_stereo ${CMAKE_CURRENT_SOURCE_DIR}/lynx_stereo.cc)
target_link_libraries(lynx_stereo orbslam3)
if(LYNX_BACKEND STREQUAL "ref")
    target_compile_definitions(lynx_stereo PRIVATE LYNX_BACKEND_REF) # replay of recorded DSP output
endif()

# ATE evaluator (Eigen only).
add_executable(eval_ate ${CMAKE_CURRENT_SOURCE_DIR}/eval_ate.cc)
target_include_directories(eval_ate PRIVATE ${SRC}/Thirdparty/eigen-3.4.0)

# 6DoF tracker for the PICO 4 (tracking cameras -> ORB-SLAM3 -> Monado pico4 driver).
set(PICO4_INTERFACE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../monado-pico4/drivers/pico4 CACHE PATH "Directory with pico4_interface.h")
add_executable(pico4-slam ${CMAKE_CURRENT_SOURCE_DIR}/pico4_slam.cc ${CMAKE_CURRENT_SOURCE_DIR}/led_tracker.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/ctrl_prior.cc)
target_include_directories(pico4-slam PRIVATE ${PICO4_INTERFACE_DIR})
target_link_libraries(pico4-slam orbslam3)

# Synthetic test of the controller LED tracker (led_test <pico4-lynx.yaml> <left.json> <right.json>).
add_executable(led_test ${CMAKE_CURRENT_SOURCE_DIR}/led_test.cc ${CMAKE_CURRENT_SOURCE_DIR}/led_tracker.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/ctrl_prior.cc)
target_include_directories(led_test PRIVATE ${SRC}/Thirdparty/eigen-3.4.0 ${OpenCV_INCLUDE_DIRS} ${PICO4_INTERFACE_DIR})
target_link_libraries(led_test ${OpenCV_LIBS})
