include_directories( BEFORE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${statismo_SOURCE_DIR}/modules/core/include
)

find_package( VTK REQUIRED )
include( ${VTK_USE_FILE} )
add_subdirectory( src )

if( ${BUILD_TESTING} )
  add_subdirectory( tests )
endif()

add_subdirectory( examples )

option( BUILD_WRAPPING_VTK "Build Python Wrappers for VTK (experimental)" OFF )
mark_as_advanced(BUILD_WRAPPING_VTK)

if( ${BUILD_WRAPPING_VTK} )
 add_subdirectory( wrapping )
endif()


module_headertest( VTK )

file( GLOB _VTK_hdrs
  include/*.h include/*.hxx include/*.hpp include/*.txx include/*.tpp
)
install( FILES ${_VTK_hdrs}
  DESTINATION ${INSTALL_INCLUDE_DIR}/statismo/VTK
)
