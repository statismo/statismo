include_directories( BEFORE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${statismo_SOURCE_DIR}/modules/core/include
  )

if(VTK_MAJOR_VERSION GREATER 5)
  set( required_vtk_modules
    vtkCommonComputationalGeometry
    vtkCommonCore
    vtkCommonDataModel
    vtkCommonExecutionModel
    vtkCommonMath
    vtkCommonMisc
    vtkCommonSystem
    vtkCommonTransforms
    vtkFiltersCore
    vtkFiltersGeneral
    vtkIOCore
    vtkIOGeometry
    vtkIOLegacy
    vtkIOXML
    vtkIOXMLParser
    )
  if( ${BUILD_WRAPPING} )
    list( APPEND required_vtk_modules vtkWrappingPythonCore )
  endif()

  list(FIND VTK_MODULES_ENABLED "vtkIOMPIParallel" HasOMPI)
  if(NOT HasOMPI EQUAL -1)
    list(APPEND required_vtk_modules vtkIOMPIParallel)
  endif()
  find_package( VTK REQUIRED COMPONENTS ${required_vtk_modules} )

else()
  find_package( VTK REQUIRED )
endif()


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


module_headertest( VTK )

file( GLOB _VTK_hdrs
  include/*.h include/*.hxx include/*.hpp include/*.txx include/*.tpp
  )
install( FILES ${_VTK_hdrs}
  DESTINATION ${INSTALL_INCLUDE_DIR}/statismo/VTK
  )
