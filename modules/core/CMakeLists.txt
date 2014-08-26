include_directories( BEFORE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_subdirectory( src )

if( ${BUILD_TESTING} )
  add_subdirectory( tests )
endif()
