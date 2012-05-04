cmake_minimum_required (VERSION 2.6)
project (statismo)
 
#
# Compile and Install HDF5
#
include(ExternalProject)
OPTION (HDF5_USE_EXTERNAL "Use External Library Building for HDF5" 0)

set(BUILD_ITK_EXAMPLES_DEFAULT OFF)
OPTION(BUILD_ITK_EXAMPLES "Build the itk examples directory" ${BUILD_ITK_EXAMPLES_DEFAULT})

set(BUILD_VTK_EXAMPLES_DEFAULT OFF)
OPTION(BUILD_VTK_EXAMPLES "Build the vtk examples directory" ${BUILD_VTK_EXAMPLES_DEFAULT})

set(BUILD_ITK_WRAPPING_DEFAULT OFF)
OPTION(BUILD_ITK_WRAPPING "Build itk wrapping" ${BUILD_ITK_WRAPPING_DEFAULT})

set(BUILD_VTK_WRAPPING_DEFAULT OFF)
OPTION(BUILD_VTK_WRAPPING "Build vtk wrapping" ${BUILD_VTK_WRAPPING_DEFAULT})

set(BUILD_STATISMO_TESTS_DEFAULT OFF)
OPTION(BUILD_STATISMO_TESTS "Build tests" ${BUILD_STATISMO_TESTS_DEFAULT})

set(BUILD_REPRESENTER_TESTS_DEFAULT OFF)
OPTION(BUILD_REPRESENTER_TESTS "Build tests" ${BUILD_REPRESENTER_TESTS_DEFAULT})


# the name of the library seems to depend on the platform
IF(WIN32)
	SET(HDF5_LIBRARIES hdf5dll;hdf5_cppdll)
ELSE(WIN32)
	SET(HDF5_LIBRARIES hdf5;hdf5_cpp)
ENDIF(WIN32)

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
	SET (HDF5_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/3rdParty/HDF5/include)
	SET (HDF5_INCLUDE_DIR_CPP ${HDF5_INCLUDE_DIR}/cpp)	
	SET (HDF5_LIBRARY_DIR ${CMAKE_INSTALL_PREFIX}/3rdParty/HDF5/lib)
	INSTALL(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/3rdParty/HDF5 DESTINATION 3rdParty)
ELSE (NOT HDF5_USE_EXTERNAL)
 	FIND_PACKAGE(HDF5 COMPONENTS CXX REQUIRED NO_MODULE)
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

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/statismo-config.cmake.in  ${CMAKE_CURRENT_BINARY_DIR}/statismo-config.cmake @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/statismo-config.cmake DESTINATION .)

set(STATISMO_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})

if (BUILD_ITK_EXAMPLES)
  add_subdirectory(Examples/ITK)
endif()

if (BUILD_VTK_EXAMPLES)
  add_subdirectory(Examples/VTK)
endif()

if (BUILD_STATISMO_TESTS)
  set(BUILD_VTK_WRAPPING ON)
  add_custom_target(test  ${CMAKE_CURRENT_SOURCE_DIR}/Tests/statismoTests/runPythonTests.sh ${STATISMO_DIR}/Wrapping/SWIG ${CMAKE_CURRENT_SOURCE_DIR}/Tests/statismoTests/)  
endif()

if (BUILD_REPRESENTER_TESTS)
  add_subdirectory(Tests/representerTests)	  
endif()


if (BUILD_VTK_WRAPPING)
  add_subdirectory(Wrapping/SWIG)
endif()

if (BUILD_ITK_WRAPPING)
  add_subdirectory(Wrapping/ITK)
endif()

