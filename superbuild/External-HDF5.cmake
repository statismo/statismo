message( "External project - HDF5" )

ExternalProject_add(HDF5
  SOURCE_DIR ${CMAKE_BINARY_DIR}/HDF5
  BINARY_DIR ${CMAKE_BINARY_DIR}/HDF5-build
  URL http://www.hdfgroup.org/ftp/HDF5/releases/hdf5-1.8/hdf5-1.8.13/src/hdf5-1.8.13.tar.gz
  UPDATE_COMMAND ""
  CMAKE_ARGS
    -DCMAKE_BUILD_TYPE:STRING=${BUILD_TYPE}
    -DHDF5_ENABLE_Z_LIB_SUPPORT:BOOL=OFF
    -DHDF5_BUILD_CPP_LIB:BOOL=ON
    -DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
    -DHDF5_BUILD_TOOLS:BOOL=OFF
    -DCMAKE_INSTALL_PREFIX:PATH=${INSTALL_DEPENDENCIES_DIR}
  INSTALL_DIR ${INSTALL_DEPENDENCIES_DIR}
)

if (WIN32)
  set( HDF5_DIR ${INSTALL_DEPENDENCIES_DIR}/cmake/hdf5/ )
  add_custom_command(
    TARGET HDF5
    POST_BUILD
      COMMAND ${CMAKE_COMMAND}
        -D INSTALL_DEPENDENCIES_DIR=${INSTALL_DEPENDENCIES_DIR}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/normalize_hdf5_lib_names.cmake
    COMMENT "normalizing hdf5 library filename"
  )

  # On Windows, find_package(HDF5) with cmake 2.8.[8,9] always ends up finding
  # the dlls instead of the libs. So setting the variables explicitly for
  # dependent projects.
  set(cmake_hdf5_c_lib    -DHDF5_C_LIBRARY:FILEPATH=${INSTALL_DEPENDENCIES_DIR}/lib/hdf5.lib)
  set(cmake_hdf5_cxx_lib  -DHDF5_CXX_LIBRARY:FILEPATH=${INSTALL_DEPENDENCIES_DIR}/lib/hdf5_cpp.lib)
  set(cmake_hdf5_libs     ${cmake_hdf5_c_lib} ${cmake_hdf5_cxx_lib})

else ()
  set( HDF5_DIR ${INSTALL_DEPENDENCIES_DIR}/share/cmake/hdf5/ )
endif ()

