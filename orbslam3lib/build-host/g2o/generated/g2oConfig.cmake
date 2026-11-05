include(CMakeFindDependencyMacro)

find_dependency(Eigen3)

if (0)
  find_dependency(OpenGL)
endif()

# Find spdlog if g2o was build with support for it
if (0)
  find_dependency(spdlog)
endif()

# Only include the targets export file if it exists (for installed builds)
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/g2oTargets.cmake")
  include("${CMAKE_CURRENT_LIST_DIR}/g2oTargets.cmake")
endif()
