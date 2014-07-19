cmake_minimum_required(VERSION 2.8.10)
project(statismo)

enable_testing()

set( STATISMO_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR} )

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

include( CTest )

find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )

find_package( Boost REQUIRED )
include_directories( ${Boost_INCLUDE_DIRS} )

find_package( HDF5  COMPONENTS C CXX)

include_directories( ${HDF5_INCLUDE_DIRS} )

# On Windows, find_package(HDF5) with cmake 2.8.[8,9] always ends up finding 
# the dlls instead of the libs. We are setting the variables directly
if(WIN32)
 SET(HDF5_LIBRARY_DIRS ${INSTALL_DEPECENCIES_DIR}/lib)
 SET(HDF5_LIBRARIES hdf5.lib hdf5_cpp.lib)
endif()

foreach( _library ${HDF5_LIBRARIES} )
  get_filename_component( _tempDir${_library} ${_library} PATH )
  list( APPEND HDF5_LIBRARY_DIR ${_tempDir${_library}} )
endforeach()

option( ITK_SUPPORT "Build ITK Support" ON )

if( ${ITK_SUPPORT} MATCHES "ON" )
  find_package( ITK REQUIRED )
  include( ${ITK_USE_FILE} )

  add_subdirectory( Examples/ITK )
endif()

option( VTK_SUPPORT "Build VTK Support" ON )

if( ${VTK_SUPPORT} MATCHES "ON" )
  find_package( VTK REQUIRED )
  include( ${VTK_USE_FILE} )

  add_subdirectory( Examples/VTK )
endif()

if( ${BUILD_TESTING} )
  add_subdirectory( Tests )
endif()
