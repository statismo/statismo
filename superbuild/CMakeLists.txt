cmake_minimum_required(VERSION 3.15)
project(Statismo-Superbuild LANGUAGES C CXX)

# ----------------------------------------
# Build type settings (https://blog.kitware.com/cmake-and-the-default-build-type/)
# ----------------------------------------

# Set a default build type if none was specified
set(_default_build_type "Release")
if(NOT BUILD_TYPE AND NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "setting build type to '${_default_build_type}' as none was specified.")
  set(BUILD_TYPE "${_default_build_type}" CACHE
      STRING "Choose the type of build" FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# ----------------------------------------
# Project options
# ----------------------------------------

include(${PROJECT_SOURCE_DIR}/../cmake/statismoOptions.cmake)

option(BUILD_SHARED_LIBS "Build shared libraries" ON)
option(USE_SYSTEM_VTK "Use system libraries for VTK" OFF)
option(USE_SYSTEM_ITK "Use system libraries for ITK" OFF)

# If itk support is enabled, we use ITK hdf5 and eigen3 to achieve more stability
include(CMakeDependentOption)
cmake_dependent_option(USE_SYSTEM_HDF5 "Use system libraries for hdf5" OFF
  "NOT ITK_SUPPORT" OFF
)
cmake_dependent_option(USE_SYSTEM_Eigen "Use system libraries for eigen3" OFF
  "NOT ITK_SUPPORT" OFF
)

set(VTK_EXTRA_OPTIONS ${CMAKE_BINARY_DIR}/INSTALL CACHE STRING "Build VTK with extra options")
set(ITK_EXTRA_OPTIONS ${CMAKE_BINARY_DIR}/INSTALL CACHE STRING "Build ITK with extra options")

# ----------------------------------------
# Project settings
# ----------------------------------------
# set CMAKE_MODULE_PATH for cmake macro/function and modules
set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  ${CMAKE_CURRENT_SOURCE_DIR}/../cmake
)

if(APPLE)
  include(CMakeOSXVariables)
endif()

set(BUILD_TYPE ${CMAKE_BUILD_TYPE})
if(NOT INSTALL_DEPENDENCIES_DIR)
    set(INSTALL_DEPENDENCIES_DIR ${CMAKE_BINARY_DIR}/INSTALL CACHE STRING "Install directory for dependencies")
endif()
set(Patches_DIR ${CMAKE_CURRENT_SOURCE_DIR}/patches)
set(Statismo_DEPENDENCIES)

# ----------------------------------------
# Project dependencies
# ----------------------------------------

include(ExternalProject)

if(NOT ITK_SUPPORT)
  if(NOT USE_SYSTEM_HDF5)
    include(External-HDF5.cmake)
    set(Statismo_DEPENDENCIES ${Statismo_DEPENDENCIES} HDF5)
  else()
    find_package(HDF5 1.10.2 REQUIRED COMPONENTS C CXX)
  endif()

  if(NOT USE_SYSTEM_Eigen)
    include(External-Eigen.cmake)
    set(Statismo_DEPENDENCIES ${Statismo_DEPENDENCIES} Eigen3)
  else()
    find_package(Eigen3 3.0.0 REQUIRED)
  endif()
endif()

if(VTK_SUPPORT)
  if(NOT USE_SYSTEM_VTK)
    include(External-VTK.cmake)
    set(Statismo_DEPENDENCIES ${Statismo_DEPENDENCIES} VTK)
  else()
    find_package(VTK 8.0.0 REQUIRED)
    include(${VTK_USE_FILE})
  endif()
endif()

if(ITK_SUPPORT)
  if(NOT USE_SYSTEM_ITK)
    include(External-ITK.cmake)
    set(Statismo_DEPENDENCIES ${Statismo_DEPENDENCIES} ITK)
  else()
    find_package(ITK 5.0.0 REQUIRED)
    include(${ITK_USE_FILE})
  endif()
endif()

ExternalProject_Add(Statismo
  DEPENDS ${Statismo_DEPENDENCIES}
  DOWNLOAD_COMMAND ""
  UPDATE_COMMAND ""
  PATCH_COMMAND ""
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..
  BINARY_DIR Statismo-build
  CMAKE_GENERATOR ${EP_CMAKE_GENERATOR}
  CMAKE_ARGS
    ${ep_common_args}
    ${CMAKE_HDF5_LIBS}
    -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
    -DCMAKE_BUILD_TYPE:STRING=${BUILD_TYPE}
    -DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
    -DBUILD_DOCUMENTATION:BOOL=${BUILD_DOCUMENTATION}
    -DBUILD_TESTS:BOOL=${BUILD_TESTS}
    -DBUILD_EXAMPLES:BOOL=${BUILD_EXAMPLES}
    -DBUILD_CLI_TOOLS:BOOL=${BUILD_CLI_TOOLS}
    -DBUILD_WRAPPING:BOOL=${BUILD_WRAPPING}
    -DBUILD_CLI_TOOLS_DOC:BOOL=${BUILD_CLI_TOOLS_DOC}
    -DBUILD_LONG_RUNNING_CLI_TESTS:BOOL=${BUILD_LONG_RUNNING_CLI_TESTS}
    -DITK_SUPPORT:BOOL=${ITK_SUPPORT}
    -DVTK_SUPPORT:BOOL=${VTK_SUPPORT}
    -DINSTALL_DEPENDENCIES_DIR:PATH=${INSTALL_DEPENDENCIES_DIR}
    -DITK_DIR:PATH=${ITK_DIR}
    -DVTK_DIR:PATH=${VTK_DIR}
    -DHDF5_DIR:PATH=${HDF5_DIR}
    -DEIGEN3_INCLUDE_DIR:PATH=${EIGEN3_INCLUDE_DIR}
  INSTALL_COMMAND ""
)
