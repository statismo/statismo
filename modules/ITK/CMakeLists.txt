include_directories( BEFORE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${statismo_SOURCE_DIR}/modules/core/include
)

if( ${BUILD_TESTING} )
  add_subdirectory( tests )
endif()

add_subdirectory( examples )

module_headertest( ITK )
