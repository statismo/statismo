include_directories( BEFORE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${statismo_SOURCE_DIR}/modules/core/include
)

if( ${BUILD_TESTING} )
  add_subdirectory( tests )
endif()

if( ${BUILD_EXAMPLES} )
  add_subdirectory( examples )
endif()

if( ${BUILD_CLI_TOOLS} )
  add_subdirectory( cli )
endif()

module_headertest( ITK )

file( GLOB _ITK_hdrs
  include/*.h include/*.hxx include/*.hpp include/*.txx include/*.tpp
)
install( FILES ${_ITK_hdrs}
  DESTINATION ${INSTALL_INCLUDE_DIR}/statismo/ITK
)
