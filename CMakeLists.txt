cmake_minimum_required(VERSION 2.8.10)
project(statismo)

if( POLICY CMP0042 )
  cmake_policy( SET CMP0042 NEW )
endif()

enable_testing()

set( STATISMO_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR} )

set(STATISMO_MAJOR_VERSION 0)
set(STATISMO_MINOR_VERSION 11)
set(STATISMO_PATCH_VERSION 0)
set(STATISMO_VERSION ${STATISMO_MAJOR_VERSION}.${STATISMO_MINOR_VERSION}.${STATISMO_PATCH_VERSION} )

set( statismo_LIB_VERSION ${STATISMO_VERSION} )
set( statismo_LIB_SOVERSION ${STATISMO_MAJOR_VERSION}.${STATISMO_MINOR_VERSION} )
set( statismo_LIB_TYPE SHARED )


# Setup build locations.
if(NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
endif()

# set CMAKE_MODULE_PATH for cmake macro/function and modules
set( CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake
)

if( APPLE )
  include( CMakeOSXVariables )
endif()

if( CMAKE_HOST_WIN32 )
  string( LENGTH "${CMAKE_CURRENT_SOURCE_DIR}" n )
  if( n GREATER 50 )
    message( FATAL_ERROR
      "Statismo source code directory path length is too long (${n} > 50)."
      "Please move the Statismo source code directory to a directory with a shorter path."
    )
  endif()

  string( LENGTH "${CMAKE_CURRENT_BINARY_DIR}" n )
  if( n GREATER 50 )
    message( FATAL_ERROR
      "Statismo build directory path length is too long (${n} > 50)."
      "Please move the Statismo build directory to a directory with a shorter path."
    )
  endif()
endif()

# installation
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

include( CTest )

include( ModuleHeaderTest )

find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )

if( WIN32 )
  set( Boost_USE_STATIC_LIBS ON )
endif()

set( REQUIRED_BOOST_COMPONENTS thread system date_time filesystem )

option( BUILD_CLI_TOOLS "Build command-line tools" ON )
#make the option to build the cli documentation available if and only if the cli tools will be built
include(CMakeDependentOption)
cmake_dependent_option( BUILD_CLI_TOOLS_DOC "Build documentation for the command-line tools" ON
  "BUILD_CLI_TOOLS" OFF
)
#it's only possible to enable the long running CLI tests if testing is enabled and the CLI tools will be built.
cmake_dependent_option( BUILD_LONG_RUNNING_CLI_TESTS "Run the cli examples(the execution of these tests can take some time)" OFF
  "BUILD_CLI_TOOLS;BUILD_TESTING" OFF
)
if( ${BUILD_CLI_TOOLS} MATCHES "ON" )
  if( ${BUILD_CLI_TOOLS_DOC} MATCHES "ON" )
    #search for pandoc to generate man pages for the cli tools
    find_program( PANDOC NAMES pandoc )
    mark_as_advanced( PANDOC )
  endif()

  #the cli tools require the program_options component from boost
  set( REQUIRED_BOOST_COMPONENTS ${REQUIRED_BOOST_COMPONENTS} program_options )
endif()

find_package( Boost 1.50 COMPONENTS ${REQUIRED_BOOST_COMPONENTS} REQUIRED )
include_directories( ${Boost_INCLUDE_DIRS} )

link_directories( ${Boost_LIBRARY_DIRS} )
add_definitions( -DBOOST_THREAD_VERSION=3 -DBOOST_FILESYSTEM_VERSION=3 )


if(WIN32)
  # On Windows, we can only build static libraries, as the dynamic libraries would be
  # missing the accompanying .lib (unless the code is entirely rewritten with __declspec(dllexport) etc.)
  # See http://www.cmake.org/Wiki/BuildingWinDLL

  set(statismo_LIB_TYPE STATIC)
endif()

option( ITK_SUPPORT "Build ITK Support" ON )

if( ${ITK_SUPPORT} MATCHES "ON" )
  find_package( ITK REQUIRED PATHS ${INSTALL_DEPENDENCIES_DIR})
  include( ${ITK_USE_FILE} )
endif()

# We rely on the hdf5 libraries that are shipped with ITK, as this will reduce version conflicts.
set(ITK_VERSION ${ITK_VERSION_MAJOR}.${ITK_VERSION_MINOR})
find_path(ITK_INCLUDE_WITH_HDF5 NAMES itkhdf5/hdf5.h PATHS ${ITK_INCLUDE_DIRS} )
set(ITK_HDF5_DIR ${ITK_INCLUDE_WITH_HDF5}/itkhdf5)
set(HDF5_INCLUDE_DIRS ${ITK_HDF5_DIR} ${ITK_HDF5_DIR}/cpp)
set(HDF5_LIBRARIES ${ITKHDF5_LIBRARIES})


set(HDF5_LIBRARY_DIR ${INSTALL_DEPENDENCIES_DIR}/lib)
include_directories( ${HDF5_INCLUDE_DIRS} )


option( VTK_SUPPORT "Build VTK Support" ON )

if( ${VTK_SUPPORT} MATCHES "ON" )
  find_package( VTK REQUIRED PATHS ${INSTALL_DEPENDENCIES_DIR})
  include( ${VTK_USE_FILE} )
endif()

option( BUILD_EXAMPLES "Build examples" ON )


option( BUILD_WRAPPING "Build Python Wrappers (experimental)" OFF )
mark_as_advanced(BUILD_WRAPPING)

#make sure the executables have access to the dll's created by the superbuild on windows to make them runnable even without installing & adding the isntall path to the cmd PATH
if( NOT INSTALL_DEPENDENCIES_DIR )
  set( TESTS_WORKING_DIR "." CACHE INTERNAL "")
else()
  set( TEST_WORKING_DIR "${INSTALL_DEPENDENCIES_DIR}/bin" CACHE INTERNAL "")
endif()

add_subdirectory( modules )
add_subdirectory( doc )
# -------------------------------------------------------------

configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/scripts/run_astyle.sh.in
  ${CMAKE_CURRENT_BINARY_DIR}/run_astyle.sh
  @ONLY
)

# -------------------------------------------------------------

# Add all targets to the build-tree export set
set( _targets statismo_core )

if( VTK_SUPPORT )
  set( _targets ${_targets} statismo_VTK )
endif()

export( TARGETS ${_targets}
  FILE "${statismo_BINARY_DIR}/statismoTargets.cmake"
)

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE statismo)

# -------------------------------------------------------------
# for the build tree
set( CONF_INCLUDE_DIRS
  "${statismo_SOURCE_DIR}/modules/core/include"
)
set( statismo_LIBRARIES statismo_core )

if( ${VTK_SUPPORT} )
  set( CONF_INCLUDE_DIRS "${CONF_INCLUDE_DIRS}" "${statismo_SOURCE_DIR}/modules/VTK/include" )
  set( statismo_LIBRARIES ${statismo_LIBRARIES} statismo_VTK )
endif()

if( ${ITK_SUPPORT} )
  set( CONF_INCLUDE_DIRS "${CONF_INCLUDE_DIRS}" "${statismo_SOURCE_DIR}/modules/ITK/include" )
endif()

configure_file( ${statismo_SOURCE_DIR}/statismoConfig.cmake.in
  ${statismo_BINARY_DIR}/statismoConfig.cmake
  @ONLY
)

# for the install tree
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}"
   "${INSTALL_INCLUDE_DIR}")

set( CONF_INCLUDE_DIRS
  "\${statismo_CMAKE_DIR}/${REL_INCLUDE_DIR}/statismo/core"
)
if( ${VTK_SUPPORT} )
  set( CONF_INCLUDE_DIRS "${CONF_INCLUDE_DIRS}" "\${statismo_CMAKE_DIR}/${REL_INCLUDE_DIR}/statismo/VTK" )
endif()

if( ${ITK_SUPPORT} )
  set( CONF_INCLUDE_DIRS "${CONF_INCLUDE_DIRS}" "\${statismo_CMAKE_DIR}/${REL_INCLUDE_DIR}/statismo/ITK" )
endif()

configure_file( ${statismo_SOURCE_DIR}/statismoConfig.cmake.in
  ${statismo_BINARY_DIR}/cmake/statismoConfig.cmake
  @ONLY
)
configure_file( ${statismo_SOURCE_DIR}/statismoConfigVersion.cmake.in
  ${statismo_BINARY_DIR}/statismoConfigVersion.cmake
  @ONLY
)
install( FILES
    "${statismo_BINARY_DIR}/cmake/statismoConfig.cmake"
    "${statismo_BINARY_DIR}/statismoConfigVersion.cmake"
  DESTINATION ${INSTALL_CMAKE_DIR} COMPONENT dev
)
install( EXPORT statismoTargets
  DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev
)
install( FILES ReadMe.md LICENSE
  DESTINATION ${INSTALL_DOC_DIR}
)
