#
# Build the documentation
#
include( ${CMAKE_ROOT}/Modules/Documentation.cmake )

mark_as_advanced( BUILD_DOCUMENTATION )

if( ${BUILD_DOCUMENTATION} )

find_package( LATEX )

  if( NOT LATEX_COMPILER )
    message( "Warning: LaTeX not found. Formulas will not be generated in documentation." )
  endif()

  configure_file( ${CMAKE_CURRENT_SOURCE_DIR}/classdoc/Doxyfile.in
    ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    @ONLY
  )

  add_custom_target(Documentation ALL
    COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    DEPENDS         ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    )

  install( DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html/
    DESTINATION ${INSTALL_DOC_DIR}/html
  )
endif()

