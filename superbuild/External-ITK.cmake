message( "External project - ITK" )

find_package(Git)
if(NOT GIT_FOUND)
  message(ERROR "Cannot find git. git is required for Superbuild")
endif()

option( USE_GIT_PROTOCOL "If behind a firewall turn this off to use http instead." ON)

set(git_protocol "git")
if(NOT USE_GIT_PROTOCOL)
  set(git_protocol "http")
endif()

set( ITK_DEPENDENCIES VTK )

if( ${USE_SYSTEM_HDF5} MATCHES "OFF" )
  set( ITK_DEPENDENCIES HDF5 ${ITK_DEPENDENCIES} )
endif()

# On Windows, find_package(HDF5) with cmake 2.8.[8,9] always ends up finding 
# the dlls instead of the libs. So setting the variables explicitly for
# dependent projects.
if(WIN32)
	  SET(cmake_hdf5_c_lib -DHDF5_C_LIBRARY:FILEPATH=${INSTALL_DEPECENCIES_DIR}/lib/hdf5.lib)
	  SET(cmake_hdf5_cxx_lib -DHDF5_CXX_LIBRARY:FILEPATH=${INSTALL_DEPECENCIES_DIR}/lib/hdf5_cpp.lib)
endif()


ExternalProject_Add(ITK
  DEPENDS ${ITK_DEPENDENCIES}
  GIT_REPOSITORY ${git_protocol}://itk.org/ITK.git
  GIT_TAG v4.5.0
  SOURCE_DIR ITK
  BINARY_DIR ITK-build
  UPDATE_COMMAND ""
  PATCH_COMMAND ""
  CMAKE_GENERATOR ${EP_CMAKE_GENERATOR}
  CMAKE_ARGS
    ${ep_common_args}
	${cmake_hdf5_c_lib}
	${cmake_hdf5_cxx_lib}
    -DBUILD_EXAMPLES:BOOL=OFF
    -DBUILD_SHARED_LIBS:BOOL=ON
    -DBUILD_TESTING:BOOL=OFF
    -DCMAKE_BUILD_TYPE:STRING=${BUILD_TYPE}
    -DITK_BUILD_DEFAULT_MODULES:BOOL=ON
    -DModule_ITKVtkGlue:BOOL=ON
    -DModule_ITKReview:BOOL=ON
    -DITK_LEGACY_REMOVE:BOOL=ON
    -DCMAKE_INSTALL_PREFIX:PATH=${INSTALL_DEPECENCIES_DIR}
    -DITK_USE_SYSTEM_HDF5:BOOL=ON
    -DHDF5_DIR:PATH=${HDF5_DIR}
    -DVTK_DIR:PATH=${VTK_DIR}
)

set( ITK_DIR ${INSTALL_DEPECENCIES_DIR}/lib/cmake/ITK-4.5/ )
