cmake_minimum_required( VERSION 2.8.10 )

project( Statismo-Superbuild )

include( ExternalProject )

set( BUILD_TYPE Release )
set( INSTALL_DEPECENCIES_DIR ${CMAKE_BINARY_DIR}/INSTALL )
set( Statismo_DEPENDENCIES )

option( USE_SYSTEM_Boost "Use system libraries for Boost" OFF )
if( ${USE_SYSTEM_Boost} MATCHES "OFF" )
  include( External-Boost.cmake )
  set( Statismo_DEPENDENCIES ${Statismo_DEPENDENCIES} Boost )
else()
endif()

option( USE_SYSTEM_Eigen "Use system libraries for Eigen" OFF )
if( ${USE_SYSTEM_Eigen} MATCHES "OFF" )
  include( External-Eigen.cmake )
  set( Statismo_DEPENDENCIES ${Statismo_DEPENDENCIES} Eigen3 )
else()
endif()

option( USE_SYSTEM_HDF5 "Use system libraries for HDF5" OFF )
if( ${USE_SYSTEM_HDF5} MATCHES "OFF" )
  include( External-HDF5.cmake )
  set( Statismo_DEPENDENCIES ${Statismo_DEPENDENCIES} HDF5 )
else()
endif()

option( USE_SYSTEM_VTK "Use system libraries for VTK" OFF )
if( ${USE_SYSTEM_VTK} MATCHES "OFF" )
  include( External-VTK.cmake )
  set( Statismo_DEPENDENCIES ${Statismo_DEPENDENCIES} VTK )
else()
endif()

option( USE_SYSTEM_ITK "Use system libraries for ITK" OFF )
if( ${USE_SYSTEM_ITK} MATCHES "OFF" )
  include( External-ITK.cmake )
  set( Statismo_DEPENDENCIES ${Statismo_DEPENDENCIES} ITK )
else()
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
    -DBoost_INCLUDE_DIR:PATH=${Boost_INCLUDE_DIR}
    -DITK_DIR:PATH=${ITK_DIR}
    -DVTK_DIR:PATH=${VTK_DIR}
    -DHDF5_DIR:PATH=${HDF5_DIR}
    -DEIGEN3_INCLUDE_DIR:PATH=${EIGEN3_INCLUDE_DIR}
    -DBUILD_ITK_EXAMPLES:BOOL=ON
    -DBUILD_VTK_EXAMPLES:BOOL=ON
    -DBUILD_STATISMO_TESTS:BOOL=ON
    -DBUILD_REPRESENTER_TESTS:BOOL=ON
  INSTALL_COMMAND ""
)
