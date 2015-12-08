#the idea here is to have an X.cxx and a X.md where X denotes the cli-command name.
set( _cli_files
  statismo-build-shape-model
  statismo-build-deformation-model
  statismo-build-gp-model
  statismo-sample
  statismo-reduce-model
  statismo-fit-surface
  statismo-fit-image
  statismo-posterior
  statismo-warp-image
)

# fix for (some?) Windows compilers which otherwise don't find utils/*.h includes
include_directories( BEFORE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

foreach( e ${_cli_files} )
  add_executable( ${e} "${e}.cxx" )
  target_link_libraries( ${e} statismo_core ${ITK_LIBRARIES} ${Boost_LIBRARIES} )
  install(
  TARGETS ${e}
  DESTINATION ${INSTALL_BIN_DIR}
  )
endforeach()

if( ${BUILD_CLI_TOOLS_DOC} MATCHES "ON" )
  if( PANDOC )
    message( STATUS "Found pandoc - generating documentation for the CLI tools" )
    #configuring man page install dir
    if( WIN32 AND NOT CYGWIN )
      #put the directory with the manuals in the same directory as the binaries on windows
      set( CLIDOC_INSTALL_DIR ${INSTALL_BIN_DIR} )
      set( CLIDOC_OUTPUT_DIR "${CMAKE_BINARY_DIR}/doc-cli" )
    else()
      #use cmake to find the man pages directory on *NIX
      include( GNUInstallDirs )
      if( CMAKE_INSTALL_MANDIR )
        set( CLIDOC_INSTALL_DIR ${CMAKE_INSTALL_MANDIR} )
      else()
        set( CLIDOC_INSTALL_DIR ${INSTALL_BIN_DIR} )
        message( STATUS "Couldn't find the man install directory - man pages will NOT be installed properly" )
      endif()
        set( CLIDOC_OUTPUT_DIR "${CMAKE_BINARY_DIR}/man8" )
    endif()

    set( CLIDOC_TARGET_DEPENDENCIES )
    set( CLIDOC_TARGET_SOURCES )

    foreach( e ${_cli_files} )
      set( CLIDOC_MD_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/${e}.md )
      if( WIN32 AND NOT CYGWIN)
        set( CLIDOC_OUT_FILE ${CLIDOC_OUTPUT_DIR}/${e}.html )
      else()
        set( CLIDOC_OUT_FILE ${CLIDOC_OUTPUT_DIR}/${e}.8 )
      endif()

      add_custom_command(
        OUTPUT ${CLIDOC_OUT_FILE}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CLIDOC_OUTPUT_DIR}
        # a '/' instead of a '\' in the input-file's path on windows leads to pandoc failing to automatically recoginze the file format -> we have specify it manually
        COMMAND ${PANDOC} -f markdown -s ${CLIDOC_MD_SOURCE} -o ${CLIDOC_OUT_FILE}
        MAIN_DEPENDENCY ${CLIDOC_MD_SOURCE}
      )

      list( APPEND CLIDOC_TARGET_DEPENDENCIES ${CLIDOC_OUT_FILE} )
      list( APPEND CLIDOC_TARGET_SOURCES ${CLIDOC_MD_SOURCE} )
    endforeach()

    #build the cli documentation by default
    add_custom_target(
      doc-cli ALL
      DEPENDS ${CLIDOC_TARGET_DEPENDENCIES}
      SOURCES ${CLIDOC_TARGET_SOURCES}
    )
    #install the entire cli documentation directory
    install(
      DIRECTORY ${CLIDOC_OUTPUT_DIR}
      DESTINATION ${CLIDOC_INSTALL_DIR}
    )
  else()
    message( STATUS "Pandoc not found - documentation will not be generated for the CLI tools" )
  endif()
endif()

if( ${BUILD_LONG_RUNNING_CLI_TESTS} )
  add_subdirectory( test )
endif()
