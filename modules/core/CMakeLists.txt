include_directories( BEFORE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${Boost_INCLUDE_DIRS}
)

add_subdirectory( src )

if( ${BUILD_TESTING} )
  add_subdirectory( tests )
endif()

module_headertest( core )

file( GLOB _core_hdrs
  include/*.h include/*.hxx include/*.hpp include/*.txx include/*.tpp
)
install( FILES ${_core_hdrs}
  DESTINATION ${INSTALL_INCLUDE_DIR}/statismo/core
)
