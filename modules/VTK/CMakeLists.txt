set(STATISMO_MODULE_NAME "VTK")

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

if( ${BUILD_EXAMPLES} )
  add_subdirectory( examples )
endif()

if( ${BUILD_WRAPPING} )
 add_subdirectory( wrapping )
endif()


module_headertest( ${STATISMO_MODULE_NAME} )

install( DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/statismo/${STATISMO_MODULE_NAME}
    DESTINATION ${INSTALL_INCLUDE_DIR}/statismo/
    )
