# normlize the file name of the hdf5 libs (remove any _D suffix that can come from visual studio debug builds)
# We copy the file, rather than simply renaming it, cause otherwise CMake gets confused.
if(WIN32)
  if(EXISTS ${INSTALL_DEPENDENCIES_DIR}/lib/hdf5_D.lib)

    configure_file(
      ${INSTALL_DEPENDENCIES_DIR}/lib/hdf5_D.lib
      ${INSTALL_DEPENDENCIES_DIR}/lib/hdf5.lib
      COPYONLY
    )

    configure_file(
      ${INSTALL_DEPENDENCIES_DIR}/lib/hdf5_cpp_D.lib
      ${INSTALL_DEPENDENCIES_DIR}/lib/hdf5_cpp.lib
      COPYONLY
    )
  endif()
endif()
