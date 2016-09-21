if(MSVC11) #i.e. Visual Studio 2012
  # Fix for VS2012 that has _VARIADIC_MAX set to 5. Don't set too high because it increases compiler memory usage / compile-time.
  add_definitions( -D_VARIADIC_MAX=10 )
  # Fix for another VS2012 problem: not all TR1 options are automatically detected, therefore we force them here.
  add_definitions( -D BOOST_HAS_TR1 )
  add_definitions( -D BOOST_NO_0X_HDR_INITIALIZER_LIST )
endif()

include( ${ITK_USE_FILE} )

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
