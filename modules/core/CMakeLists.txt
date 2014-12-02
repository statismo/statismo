set(STATISMO_MODULE_NAME "core")

include_directories( BEFORE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_subdirectory( src )

if( ${BUILD_TESTING} )
  add_subdirectory( tests )
endif()

module_headertest( ${STATISMO_MODULE_NAME} )

install( DIRECTORY ${CMAKE_CURREND_SOURCE_DIR}/include/statismo/${STATISMO_MODULE_NAME}
    DESTINATION ${INSTALL_INCLUDE_DIR}/statismo/
)
