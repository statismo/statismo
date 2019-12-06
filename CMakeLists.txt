cmake_minimum_required(VERSION 3.15)
project(statismo VERSION 0.11.1 LANGUAGES C CXX)

# ----------------------------------------
# Project options
# ----------------------------------------

include(${PROJECT_SOURCE_DIR}/cmake/statismoOptions.cmake)

# ----------------------------------------
# Project settings
# ----------------------------------------

set(STATISMO_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(statismo_LIB_VERSION ${STATISMO_VERSION})
set(statismo_LIB_SOVERSION ${STATISMO_MAJOR_VERSION}.${STATISMO_MINOR_VERSION})
set(statismo_LIB_TYPE SHARED)

# Setup build locations.
if(NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
endif()

# set CMAKE_MODULE_PATH for cmake macro/function and modules
set(CMAKE_MODULE_PATH
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake
  ${CMAKE_MODULE_PATH}
)

if(APPLE)
  include(CMakeOSXVariables)
endif()

if(CMAKE_HOST_WIN32)
  set(STATISMO_MAX_PATH 50)
  string(LENGTH "${CMAKE_CURRENT_SOURCE_DIR}" n)
  if(n GREATER ${STATISMO_MAX_PATH})
    message(FATAL_ERROR
      "Statismo source code directory path length is too long (${n} > ${STATISMO_MAX_PATH})."
      "Please move the Statismo source code directory to a directory with a shorter path."
   )
  endif()

  string(LENGTH "${CMAKE_CURRENT_BINARY_DIR}" n)
  if(n GREATER ${STATISMO_MAX_PATH})
    message(FATAL_ERROR
      "Statismo build directory path length is too long (${n} > ${STATISMO_MAX_PATH})."
      "Please move the Statismo build directory to a directory with a shorter path."
   )
  endif()
endif()

if(WIN32)
  # TODO: Fix this with cmake auto-generated export macro
  set(statismo_LIB_TYPE STATIC)
endif()

# ----------------------------------------
# Install instructions
# ----------------------------------------

set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH
  "Installation directory for header files")
set(INSTALL_DOC_DIR share/doc/statismo CACHE PATH
  "Installation directory for documentation")

if(WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKE_DIR CMake)
else()
  set(DEF_INSTALL_CMAKE_DIR lib/cmake/statismo)
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
  "Installation directory for CMake files")

# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()

# ----------------------------------------
# Sanity
# ----------------------------------------

# Formating
set(CLANG_FORMAT_BIN_NAME clang-format-6.0)
set(CLANG_FORMAT_EXCLUDE_PATTERNS  "build*" ${PROJECT_BINARY_DIR})
find_package(ClangFormat)

# Extra compiler options for warnings
string(
    APPEND STATISMO_COMPILE_OPTIONS
    "$<IF:$<CXX_COMPILER_ID:MSVC>,"
        "/W4;$<$<CONFIG:RELEASE>:/O2>,"
        "-Wall;-Wextra;-Werror;"
            "$<$<CONFIG:RELEASE>:-O3>;"
    ">"
)

# Clang tidy

#
# sudo ln -s /usr/bin/clang-tidy-6.0 /usr/bin/clang-tidy
#
if (BUILD_WITH_TIDY)
  find_program(
    CLANG_TIDY_EXE
    NAMES "clang-tidy"
    DOC "Path to clang-tidy executable"
  )

  if(NOT CLANG_TIDY_EXE)
    message(FATAL_ERROR "clang-tidy not found")
  else()
    message(STATUS "clang-tidy found: ${CLANG_TIDY_EXE}")
    set(WITH_CLANG_TIDY "${CLANG_TIDY_EXE}" "-checks=-*,cert-*,bugprone-*,cppcoreguidelines-*,google-*,hicpp-*,misc-*,\
    modernize-*,performance-*,readability-*,-google-readability-namespace-comments,-cert-dcl58-cpp,\
    -hicpp-special-member-functions,-cppcoreguidelines-pro-type-reinterpret-cast,-cppcoreguidelines-pro-bounds-array-to-pointer-decay,\
    -cppcoreguidelines-owning-memory,-readability-implicit-bool-conversion,-hicpp-no-array-decay,\
    -cppcoreguidelines-pro-bounds-pointer-arithmetic,-google-build-using-namespace,\
    -cppcoreguidelines-pro-bounds-constant-array-index")
  endif()
endif()

# ----------------------------------------
# Tests configuration
# ----------------------------------------

if (BUILD_TESTS)
  include_directories(${PROJECT_SOURCE_DIR}/helpers/tests)
  enable_testing()

  if (UNIX)
    find_program(MEMORYCHECK_COMMAND valgrind)
    if(MEMORYCHECK_COMMAND)
      set(MEMORYCHECK_COMMAND_OPTIONS "--trace-children=yes --leak-check=full")
      file(TO_CMAKE_PATH "${PROJECT_SOURCE_DIR}/cmake/valgrind.supp" MEMORYCHECK_SUPPRESSIONS_FILE)
    endif()
  endif()
  include(CTest)

endif()

# ----------------------------------------
# Dependencies
# ----------------------------------------
if(${BUILD_CLI_TOOLS} AND ${BUILD_CLI_TOOLS_DOC})
  find_program(PANDOC NAMES pandoc)
  mark_as_advanced(PANDOC)
endif()

if(ITK_SUPPORT)
  find_package(ITK 5.0.0 REQUIRED PATHS ${INSTALL_DEPENDENCIES_DIR})
  include(${ITK_USE_FILE})
endif()

# if possible, rely on libs shipped with itk for better comptibility
if(ITK_SUPPORT)
  set(ITK_VERSION ${ITK_VERSION_MAJOR}.${ITK_VERSION_MINOR})
  find_path(ITK_INCLUDE_WITH_HDF5 NAMES itkhdf5/hdf5.h PATHS ${ITK_INCLUDE_DIRS})
  set(ITK_HDF5_DIR ${ITK_INCLUDE_WITH_HDF5}/itkhdf5)
  set(HDF5_INCLUDE_DIRS ${ITK_HDF5_DIR} ${ITK_HDF5_DIR}/cpp)
  set(HDF5_LIBRARIES ${ITKHDF5_LIBRARIES})
  set(HDF5_LIBRARY_DIR ${INSTALL_DEPENDENCIES_DIR}/lib)

  find_path(ITK_INCLUDE_WITH_EIGEN NAMES itkeigen/Eigen/Eigen PATHS ${ITK_INCLUDE_DIRS})
  set(EIGEN3_INCLUDE_DIR ${ITK_INCLUDE_WITH_EIGEN}/itkeigen)

  include_directories(${EIGEN3_INCLUDE_DIR} ${HDF5_INCLUDE_DIRS})
else()
  # find eigen
  find_package(Eigen3 3.0.0 REQUIRED)
  include_directories(${EIGEN3_INCLUDE_DIR})
  # find hdf5
  if(BUILD_SHARED_LIBS)
    set(HDF5_COMPONENT_TYPE shared)
  else()
    set(HDF5_COMPONENT_TYPE static)
  endif()
  find_package(HDF5 1.10.2 REQUIRED PATHS ${HDF5_DIR} COMPONENTS C CXX ${HDF5_COMPONENT_TYPE})
  if(HDF5_shared_C_FOUND)
    include_directories(${HDF5_INCLUDE_DIR})
    set(HDF5_LIBRARIES hdf5::hdf5-shared hdf5::hdf5_cpp-shared)
  elseif(HDF5_static_C_FOUND)
    # Package is imported the hdf5 way
    include_directories(${HDF5_INCLUDE_DIR})
    set(HDF5_LIBRARIES hdf5::hdf5-static hdf5::hdf5_cpp-static)
  else()
    # Package is imported the cmake way
    include_directories(${HDF5_INCLUDE_DIRS})
    set(HDF5_LIBRARIES ${HDF5_C_LIBRARIES} ${HDF5_CXX_LIBRARIES})
    set(HDF5_LIBRARY_DIR ${HDF5_LIBRARY_DIRS})
  endif()
endif()

if(VTK_SUPPORT)
  set(_required_vtk_modules
    vtkCommonComputationalGeometry
    vtkCommonCore
    vtkCommonDataModel
    vtkCommonExecutionModel
    vtkCommonMath
    vtkCommonMisc
    vtkCommonSystem
    vtkCommonTransforms
    vtkFiltersCore
    vtkFiltersGeneral
    vtkIOCore
    vtkIOGeometry
    vtkIOLegacy
    vtkIOXML
    vtkIOXMLParser
  )
  if(${BUILD_WRAPPING})
    list(APPEND _required_vtk_modules vtkWrappingPythonCore)
  endif()

  list(FIND VTK_MODULES_ENABLED "vtkIOMPIParallel" HasOMPI)
  if(NOT HasOMPI EQUAL -1)
    list(APPEND _required_vtk_modules vtkIOMPIParallel)
  endif()
  find_package(VTK 8.0.0 REQUIRED PATHS ${INSTALL_DEPENDENCIES_DIR} COMPONENTS ${_required_vtk_modules})
  include(${VTK_USE_FILE})
endif()

# make sure the executables have access to the dll's created by the superbuild on windows 
# to make them runnable even without installing & adding the install path to the cmd PATH
if(NOT INSTALL_DEPENDENCIES_DIR)
  set(TESTS_WORKING_DIR "." CACHE INTERNAL "")
else()
  set(TEST_WORKING_DIR "${INSTALL_DEPENDENCIES_DIR}/bin" CACHE INTERNAL "")
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/contrib/boost_light)

# ----------------------------------------
# Process subdirectories
# ----------------------------------------

add_subdirectory(modules)
add_subdirectory(doc)

# ----------------------------------------
# Targets export generic settings
# ----------------------------------------

# Version file generation
include(CMakePackageConfigHelpers)

# ----------------------------------------
# Build-tree targets export
# ----------------------------------------

# Add all targets to the build-tree export set
set(_targets statismo_core)

if(VTK_SUPPORT)
  set(_targets ${_targets} statismo_VTK)
endif()

export(TARGETS ${_targets}
  FILE "${statismo_BINARY_DIR}/statismoTargets.cmake"
)

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE statismo)

set(statismo_LIBRARIES statismo::statismo_core)

if(VTK_SUPPORT)
  set(statismo_LIBRARIES ${statismo_LIBRARIES} statismo::statismo_VTK)
endif()

write_basic_package_version_file(
    ${statismo_BINARY_DIR}/statismoConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion)

configure_package_config_file(
    ${statismo_SOURCE_DIR}/cmake/statismoConfig.cmake.in
    ${statismo_BINARY_DIR}/statismoConfig.cmake
    INSTALL_DESTINATION .
)

# ----------------------------------------
# Install tree targets export
# ----------------------------------------
configure_package_config_file(
    ${statismo_SOURCE_DIR}/cmake/statismoConfig.cmake.in
    ${statismo_BINARY_DIR}/cmake/statismoConfig.cmake
    INSTALL_DESTINATION ${INSTALL_CMAKE_DIR}
)

install(FILES
    ${statismo_BINARY_DIR}/statismoConfig.cmake
    ${statismo_BINARY_DIR}/statismoConfigVersion.cmake
   DESTINATION ${INSTALL_CMAKE_DIR} COMPONENT dev
)
install(EXPORT statismoTargets NAMESPACE statismo::
  DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev
)
install(FILES ReadMe.md LICENSE
  DESTINATION ${INSTALL_DOC_DIR}
)
