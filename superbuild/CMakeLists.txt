cmake_minimum_required( VERSION 2.8.10 )

project( Statismo-Superbuild )

# set CMAKE_MODULE_PATH for cmake macro/function and modules
set( CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  ${CMAKE_CURRENT_SOURCE_DIR}/../cmake
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

option( BUILD_SHARED_LIBS "Build shared libraries" ON )

include( ExternalProject )

set( BUILD_TYPE ${CMAKE_BUILD_TYPE} )
if( NOT INSTALL_DEPENDENCIES_DIR )
    set( INSTALL_DEPENDENCIES_DIR ${CMAKE_BINARY_DIR}/INSTALL CACHE STRING "Install directory for dependencies")
endif()
set( Patches_DIR ${CMAKE_CURRENT_SOURCE_DIR}/patches )
set( Statismo_DEPENDENCIES )

option( USE_SYSTEM_Boost "Use system libraries for Boost" OFF )
if( ${USE_SYSTEM_Boost} MATCHES "OFF" )
  include( External-Boost.cmake )
  set( Statismo_DEPENDENCIES ${Statismo_DEPENDENCIES} Boost )
else()
  find_package( Boost REQUIRED )
endif()

option( USE_SYSTEM_Eigen "Use system libraries for Eigen" OFF )
if( ${USE_SYSTEM_Eigen} MATCHES "OFF" )
  include( External-Eigen.cmake )
  set( Statismo_DEPENDENCIES ${Statismo_DEPENDENCIES} Eigen3 )
else()
  find_package( Eigen3 REQUIRED )
endif()


option( USE_SYSTEM_VTK "Use system libraries for VTK" OFF )
if( ${USE_SYSTEM_VTK} MATCHES "OFF" )
  include( External-VTK.cmake )
  set( Statismo_DEPENDENCIES ${Statismo_DEPENDENCIES} VTK )
else()
  find_package( VTK REQUIRED )
  include( ${VTK_USE_FILE} )
endif()

option( USE_SYSTEM_ITK "Use system libraries for ITK" OFF )
if( ${USE_SYSTEM_ITK} MATCHES "OFF" )
  include( External-ITK.cmake )
  set( Statismo_DEPENDENCIES ${Statismo_DEPENDENCIES} ITK )
else()
  find_package( ITK REQUIRED )
  include( ${ITK_USE_FILE} )
endif()

ExternalProject_Add( Statismo
  DEPENDS ${Statismo_DEPENDENCIES}
  DOWNLOAD_COMMAND ""
  UPDATE_COMMAND ""
  PATCH_COMMAND ""
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..
  BINARY_DIR Statismo-build
  CMAKE_GENERATOR ${EP_CMAKE_GENERATOR}
  CMAKE_ARGS
    ${ep_common_args}
    ${cmake_hdf5_libs}
    -DCMAKE_BUILD_TYPE:STRING=${BUILD_TYPE}
    -DITK_SUPPORT:BOOL=ON
    -DVTK_SUPPORT:BOOL=ON
    -DBoost_INCLUDE_DIR:PATH=${Boost_INCLUDE_DIR}
    -DBOOST_ROOT:PATH=${BOOST_ROOT}
    -DINSTALL_DEPENDENCIES_DIR:PATH=${INSTALL_DEPENDENCIES_DIR}
    -DITK_DIR:PATH=${ITK_DIR}
    -DVTK_DIR:PATH=${VTK_DIR}
    -DHDF5_DIR:PATH=${HDF5_DIR}
    -DEIGEN3_INCLUDE_DIR:PATH=${EIGEN3_INCLUDE_DIR}
  INSTALL_COMMAND ""
)
