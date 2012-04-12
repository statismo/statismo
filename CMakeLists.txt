cmake_minimum_required (VERSION 2.6)
project (statismo)
 
#
# Compile and Install HDF5
#
include(ExternalProject)
OPTION (HDF5_USE_EXTERNAL "Use External Library Building for HDF5" 0)

IF (NOT HDF5_USE_EXTERNAL)
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
	  INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/3rdParty/HDF5
	) 
	INSTALL(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/3rdParty/HDF5 DESTINATION 3rdParty)
	SET (HDF5_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/3rdParty/HDF5/include)
	SET (HDF5_INCLUDE_DIR_CPP ${HDF5_INCLUDE_DIR}/cpp)	
	SET (HDF5_LIBRARY_DIR ${CMAKE_INSTALL_PREFIX}/3rdParty/HDF5/lib)
	SET (HDF5_LIBRARIES hdf5;hdf5_cpp)
ELSE (NOT HDF5_USE_EXTERNAL)
 	FIND_PACKAGE(HDF5 COMPONENTS CXX REQUIRED NO_MODULE)
 	# the library dir is often not set correclty by HDF5. If it is not set, we let the user specify it
 	FIND_LIBRARY(HDF5_LIBRARY_DIR hdf5 ${HDF5_LIBRARY_DIR})
ENDIF (NOT HDF5_USE_EXTERNAL)


#
# Install boost and eigen, by just copying them from the 3rdParty directory
#  
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/boost DESTINATION 3rdParty)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/Eigen DESTINATION 3rdParty)

#
# Install the statismo and statismo itk files (these are header only, so no compilation needed)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/statismo DESTINATION .)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/statismo_ITK DESTINATION .)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Representers DESTINATION .)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/statismo-config.cmake.in  ${CMAKE_CURRENT_BINARY_DIR}/statismo-config.cmake)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/statismo-config.cmake DESTINATION .)
