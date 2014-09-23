add_subdirectory( core )

if( ${ITK_SUPPORT} )
  add_subdirectory( ITK )
endif()

if( ${VTK_SUPPORT} )
  add_subdirectory( VTK )
endif()
