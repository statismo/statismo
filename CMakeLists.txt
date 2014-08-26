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

include( ModuleHeaderTest )

find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )

find_package( Boost REQUIRED )
include_directories( ${Boost_INCLUDE_DIRS} )

find_package( HDF5  COMPONENTS C CXX)
include_directories( ${HDF5_INCLUDE_DIRS} )

# On Windows, find_package(HDF5) with cmake 2.8.[8,9] always ends up finding
# the dlls instead of the libs. We are setting the variables directly
if(WIN32)
 set(HDF5_LIBRARY_DIRS ${INSTALL_DEPECENCIES_DIR}/lib)
 set(HDF5_LIBRARIES hdf5.lib hdf5_cpp.lib)
endif()

foreach( _library ${HDF5_LIBRARIES} )
  get_filename_component( _tempDir${_library} ${_library} PATH )
  list( APPEND HDF5_LIBRARY_DIR ${_tempDir${_library}} )
endforeach()

option( ITK_SUPPORT "Build ITK Support" ON )

if( ${ITK_SUPPORT} MATCHES "ON" )
  find_package( ITK REQUIRED )
  include( ${ITK_USE_FILE} )
endif()

option( VTK_SUPPORT "Build VTK Support" ON )

if( ${VTK_SUPPORT} MATCHES "ON" )
  find_package( VTK REQUIRED )
  include( ${VTK_USE_FILE} )
endif()

option( BUILD_SHARED_LIBS "Build shared libraries" ON )

add_subdirectory( modules )

# -------------------------------------------------------------
configure_file( ${statismo_SOURCE_DIR}/statismo-config.cmake.in
  ${statismo_BINARY_DIR}/statismo-config.cmake
  @ONLY
)

file( GLOB statismo_hdrs
  statismo/*.h statismo/*.hxx statismo/*.hpp statismo/*.txx  statismo/*.tpp statismo/*.cpp statismo/*.cxx
)
install( FILES ${statismo_hdrs}
  DESTINATION include/statismo/statismo
)

file( GLOB statismo_itk_hdrs
  statismo_ITK/*.h statismo_ITK/*.hxx statismo_ITK/*.hpp statismo_ITK/*.txx  statismo_ITK/*.tpp
)
install( FILES ${statismo_itk_hdrs}
  DESTINATION include/statismo/statismo_ITK
)

install( FILES ${statismo_BINARY_DIR}/statismo-config.cmake
  DESTINATION lib/cmake/statismo
)

file( GLOB statismo_rep
  Representers/*.h Representers/*.hxx Representers/*.hpp Representers/*.txx  Representers/*.tpp
)
install( FILES ${statismo_rep}
  DESTINATION include/statismo/Representers/
)

file( GLOB statismo_rep_itk
  Representers/ITK/*.h Representers/ITK/*.hxx Representers/ITK/*.hpp Representers/ITK/*.txx  Representers/ITK/*.tpp
)

install( FILES ${statismo_rep_itk}
  DESTINATION include/statismo/Representers/ITK
)

file( GLOB statismo_rep_vtk
  Representers/VTK/*.h Representers/VTK/*.hxx Representers/VTK/*.hpp Representers/VTK/*.txx  Representers/VTK/*.tpp
)
install( FILES ${statismo_rep_vtk}
  DESTINATION include/statismo/Representers/VTK
)

