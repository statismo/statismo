cmake_minimum_required( VERSION 2.8.10 )

project( Statismo )

include( ExternalProject )

set( BUILD_TYPE Release )
set( INSTALL_DEPECENCIES_DIR ${CMAKE_BINARY_DIR}/INSTALL )

option( USE_SYSTEM_Boost "Use system libraries for Boost" OFF )
if( ${USE_SYSTEM_Boost} MATCHES "OFF" )
  include( External-Boost.cmake )
else()
endif()

option( USE_SYSTEM_Eigen "Use system libraries for Eigen" OFF )
if( ${USE_SYSTEM_Eigen} MATCHES "OFF" )
  include( External-Eigen.cmake )
else()
endif()

option( USE_SYSTEM_HDF5 "Use system libraries for HDF5" OFF )
if( ${USE_SYSTEM_HDF5} MATCHES "OFF" )
  include( External-HDF5.cmake )
else()
endif()

option( USE_SYSTEM_VTK "Use system libraries for VTK" OFF )
if( ${USE_SYSTEM_VTK} MATCHES "OFF" )
  include( External-VTK.cmake )
else()
endif()

option( USE_SYSTEM_ITK "Use system libraries for ITK" OFF )
if( ${USE_SYSTEM_ITK} MATCHES "OFF" )
  include( External-ITK.cmake )
else()
endif()


