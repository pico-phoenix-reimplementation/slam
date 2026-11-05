# Install script for directory: /home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/iriska/picoreverse/slam/orbslam3lib/build-host/g2o/g2o/core/libg2o_core.a")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/g2o/core" TYPE FILE FILES
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/auto_differentiation.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/base_binary_edge.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/base_dynamic_vertex.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/base_edge.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/base_fixed_sized_edge.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/base_fixed_sized_edge.hpp"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/base_multi_edge.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/base_unary_edge.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/base_variable_sized_edge.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/base_variable_sized_edge.hpp"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/base_vertex.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/base_vertex.hpp"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/batch_stats.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/block_solver.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/block_solver.hpp"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/cache.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/creators.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/dynamic_aligned_buffer.hpp"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/eigen_types.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/estimate_propagator.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/factory.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/g2o_core_api.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/hyper_dijkstra.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/hyper_graph.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/hyper_graph_action.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/io_helper.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/jacobian_workspace.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/linear_solver.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/marginal_covariance_cholesky.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/matrix_operations.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/matrix_structure.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/openmp_mutex.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/optimizable_graph.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm_dogleg.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm_factory.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm_property.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/optimization_algorithm_with_hessian.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/ownership.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/parameter.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/parameter_container.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/robust_kernel.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/robust_kernel_factory.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/solver.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/sparse_block_matrix.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/sparse_block_matrix.hpp"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/sparse_block_matrix_ccs.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/sparse_block_matrix_diagonal.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/sparse_optimizer.h"
    "/home/iriska/picoreverse/slam/orbslam3lib/app/src/main/cpp/Thirdparty/g2o/g2o/core/sparse_optimizer_terminate_action.h"
    )
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/iriska/picoreverse/slam/orbslam3lib/build-host/g2o/g2o/core/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
