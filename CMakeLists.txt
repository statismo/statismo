cmake_minimum_required(VERSION 2.8.10)
project(statismo)

enable_testing()

set(STATISMO_MAJOR_VERSION 0)
set(STATISMO_MINOR_VERSION 9)
set(STATISMO_PATCH_VERSION 0)
set(STATISMO_VERSION ${STATISMO_MAJOR_VERSION}.${STATISMO_MINOR_VERSION}.${STATISMO_PATCH_VERSION} )

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

find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )

find_package( Boost REQUIRED )
include( ${Boost_INCLUDE_DIRS} )

find_package( HDF5 COMPONENTS C CXX )
include_directories( ${HDF5_INCLUDE_DIR} ${HDF5_INCLUDE_DIR_CPP} )

set( HDF5_LIBRARY_DIR )
foreach( _library ${HDF5_LIBRARIES} )
  get_filename_component( _tempDir${_library} ${_library} PATH )
  list( APPEND HDF5_LIBRARY_DIR ${_tempDir${_library}} )
endforeach()

list( REMOVE_DUPLICATES HDF5_LIBRARY_DIR )


option( ITK_SUPPORT "Build ITK Support" ON )

if( ${ITK_SUPPORT} MATCHES "ON" )
  find_package( ITK REQUIRED )
  include( ${ITK_USE_FILE} )

  add_subdirectory( Examples/ITK )
endif()

#
# Compile and Install HDF5
#

