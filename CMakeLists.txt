cmake_minimum_required (VERSION 2.6)
project (statismo)

ENABLE_TESTING()

SET(STATISMO_VERSION 0.8) 
SET(STATISMO_MINOR_VERSION 0)


# rpath settings
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib;${CMAKE_INSTALL_PREFIX}/lib/hdf5;")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Setup build locations.
if(NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
endif()


#
# Compile and Install HDF5
#
include(ExternalProject)
OPTION (HDF5_USE_EXTERNAL "Use External Library Building for HDF5" OFF)

# it seems that on linux only the shared libraries work, while on windows static seems to work fine
IF(WIN32)
	SET(HDF5_BUILD_SHARED Off)
	SET(HDF5_LIBRARIES debug hdf5d debug hdf5_cppd optimized hdf5; optimized hdf5_cpp)	
	SET (HDF5_BUILD_TYPE "RELEASE" CACHE STRING "Build type (RELEASE DEBUG) of the hdf5 library" )
ELSE(WIN32)
	SET(HDF5_BUILD_SHARED On)
	SET(HDF5_LIBRARIES debug hdf5 debug hdf5_cpp optimized hdf5; optimized hdf5_cpp)
	SET (HDF5_BUILD_TYPE "RELEASE")
ENDIF(WIN32)



IF (NOT HDF5_USE_EXTERNAL)
	ExternalProject_add(HDF5
	  SOURCE_DIR ${CMAKE_BINARY_DIR}/HDF5
	  BINARY_DIR ${CMAKE_BINARY_DIR}/HDF5-build
	  URL "${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/hdf5-1.8.7.tar.gz"
	  UPDATE_COMMAND ""
	  CMAKE_ARGS
	  -DCMAKE_BUILD_TYPE:STRING=${HDF5_BUILD_TYPE}
	  -DHDF5_ENABLE_Z_LIB_SUPPORT:BOOL=Off
	  -DHDF5_BUILD_CPP_LIB:BOOL=On
	  -DBUILD_SHARED_LIBS:BOOL=${HDF5_BUILD_SHARED}
	  -DHDF5_BUILD_TOOLS:BOOL=Off
	  -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_CURRENT_BINARY_DIR}/3rdParty/HDF5/
	  INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/3rdParty/HDF5/
	) 
	SET (HDF5_INCLUDE_DIR ${CMAKE_BINARY_DIR}/3rdParty/HDF5/include)
	SET (HDF5_INCLUDE_DIR_CPP ${HDF5_INCLUDE_DIR}/cpp)			
	SET (HDF5_LIBRARY_DIR ${CMAKE_BINARY_DIR}/3rdParty/HDF5/lib)
     
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/3rdParty/HDF5/include/ DESTINATION include/hdf5 FILES_MATCHING PATTERN "*.h") 
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/3rdParty/HDF5/include/cpp/ DESTINATION include/hdf5/cpp FILES_MATCHING PATTERN "*.h") 
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/3rdParty/HDF5/lib/ DESTINATION lib/hdf5 FILES_MATCHING PATTERN "*hdf5*") 
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/3rdParty/HDF5/bin/ DESTINATION bin FILES_MATCHING PATTERN "*hdf5*") 

SET (STATISMO_LIBRARY_DIR ${CMAKE_INSTALL_PREFIX}/lib/hdf5)


ELSE (NOT HDF5_USE_EXTERNAL)

 	FIND_PATH(HDF5_INCLUDE_DIR hdf5.h)
 	FIND_PATH(HDF5_INCLUDE_DIR_CPP H5Cpp.h)
 	FIND_LIBRARY(HDF5_LIBRARY hdf5)
 	FIND_LIBRARY(HDF5_CPP_LIBRARY hdf5_cpp)
	GET_FILENAME_COMPONENT(HDF5_LIBRARY_DIR ${HDF5_LIBRARY} PATH) 
	SET (HDF5_LIBRARIES ${HDF5_LIBRARY} ${HDF5_CPP_LIBRARY})
	SET (STATISMO_LIBRARY_DIR ${HDF5_LIBRARY_DIR})	
ENDIF (NOT HDF5_USE_EXTERNAL)


#
#optional examples and wrapping
OPTION(BUILD_ITK_EXAMPLES "Build the itk examples " OFF)
OPTION(BUILD_VTK_EXAMPLES "Build the vtk examples " OFF)
OPTION(BUILD_PYTHON_WRAPPING_ITK "Build python wrapping for the itk representers using wrapitk (experimental)" OFF)
OPTION(BUILD_PYTHON_WRAPPING_VTK "Build python wrapping for the VTK
representers" OFF)

MARK_AS_ADVANCED(BUILD_PYTHON_WRAPPING_ITK)

#
#optional tests
#
OPTION(BUILD_STATISMO_TESTS "Build tests" OFF)
OPTION(BUILD_STATISMO_TESTS_PYTHON "Build Unit tests written in python (requires python wrapping)") 
OPTION(BUILD_REPRESENTER_TESTS "Build representer tests (requires ITK and VTK)" OFF)
MARK_AS_ADVANCED(BUILD_REPRESENTER_TESTS)


#
# Install boost and eigen, by just copying them from the 3rdParty directory
#  
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/boost DESTINATION include/)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/Eigen DESTINATION include/)

#
# Install the statismo and statismo itk files (these are header only, so no compilation needed)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/statismo DESTINATION include)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/statismo_ITK DESTINATION include)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/Representers DESTINATION include)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/statismo-config.cmake.in  ${CMAKE_CURRENT_BINARY_DIR}/statismo-config.cmake @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/statismo-config.cmake DESTINATION lib/cmake/statismo-${STATISMO_VERSION})

# 
# copy the data
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/data DESTINATION share)

set(STATISMO_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})

if (BUILD_ITK_EXAMPLES)
  add_subdirectory(Examples/ITK)
endif()

if (BUILD_VTK_EXAMPLES)
  add_subdirectory(Examples/VTK)
endif()

if (BUILD_STATISMO_TESTS)
  add_subdirectory(Tests)
endif()


if (BUILD_STATISMO_TESTS_PYTHON)
  set(BUILD_PYTHON_WRAPPING_VTK ON)
  add_custom_target(unit_test  ${CMAKE_CURRENT_SOURCE_DIR}/Tests/statismoTests/runPythonTests.sh ${CMAKE_CURRENT_BINARY_DIR}/Wrapping/SWIG ${CMAKE_CURRENT_SOURCE_DIR}/Tests/statismoTests/)
endif()

if (BUILD_TESTS)
  add_subdirectory(Tests)	  
endif()


if (BUILD_REPRESENTER_TESTS)
  add_subdirectory(Tests/representerTests)	  
endif()


if (BUILD_PYTHON_WRAPPING_VTK)
  add_subdirectory(Wrapping/SWIG)
endif()

if (BUILD_PYTHON_WRAPPING_ITK)
  add_subdirectory(Wrapping/ITK)
endif()


