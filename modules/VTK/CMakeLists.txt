include_directories( BEFORE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${statismo_SOURCE_DIR}/modules/core/include
)

add_subdirectory( src )

if( ${BUILD_TESTING} )
  add_subdirectory( tests )
endif()

add_subdirectory( examples )

module_headertest( VTK )

file( GLOB _VTK_hdrs
  include/*.h include/*.hxx include/*.hpp include/*.txx include/*.tpp
)
install( FILES ${_VTK_hdrs}
  DESTINATION ${INSTALL_INCLUDE_DIR}/statismo/VTK
)
