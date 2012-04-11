cmake_minimum_required (VERSION 2.6)
project (statismo)
 
#
# Compile and Install HDF5
#
include(ExternalProject)


ExternalProject_add(HDF5
  SOURCE_DIR HDF5
  BINARY_DIR HDF5-build
  URL "${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/hdf5-1.8.7.tar.gz"
  UPDATE_COMMAND ""
  CMAKE_ARGS
  -DCMAKE_BUILD_TYPE:STRING=Release
  -DHDF5_ENABLE_Z_LIB_SUPPORT:BOOL=Off
  -DHDF5_BUILD_CPP_LIB:BOOL=On
  -DBUILD_SHARED_LIBS:BOOL=On
  -DHDF5_BUILD_TOOLS:BOOL=On
  -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_CURRENT_BINARY_DIR}/3rdParty/HDF5
  INSTALL_DIR ${HDF5_INSTALL_PREFIX}
) 

INSTALL(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/3rdParty/HDF5 DESTINATION 3rdParty)   
 
#
# Install boost and eigen, by just copying them from the 3rdParty directory
#  
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/boost DESTINATION 3rdParty)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/Eigen DESTINATION 3rdParty)

#
# Install the statismo and statismo itk files (these are header only, so no compilation needed)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/statismo DESTINATION .)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/statismo_ITK DESTINATION .)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/statismo-config.cmake DESTINATION .)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Representers DESTINATION .)
