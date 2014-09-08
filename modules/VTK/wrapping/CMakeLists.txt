find_package( PythonLibs REQUIRED )
include_directories(${PYTHON_INCLUDE_PATH})

if( VTK_MAJOR_VERSION EQUAL 5 )
  # VTK 5 needs additional libraries
  set( VTK_LIBRARIES_PYTHON
    vtkCommonPythonD
    vtkPythonCore
  )
else()
  set( VTK_LIBRARIES_PYTHON "" )
endif()

link_directories(
  ${HDF5_LIBRARY_DIR}
)

find_package( SWIG REQUIRED )
include( ${SWIG_USE_FILE} )

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set_source_files_properties(statismo.i PROPERTIES CPLUSPLUS ON)
set_source_files_properties(statismo.i PROPERTIES SWIG_FLAGS "-includeall")

file( GLOB _Statismo_src
  ${statismo_SOURCE_DIR}/modules/VTK/src/*.cxx
)

swig_add_module( statismo_VTK python statismo.i ${_Statismo_src})

swig_link_libraries( statismo_VTK ${PYTHON_LIBRARIES}  statismo-core ${VTK_LIBRARIES} ${VTK_LIBRARIES_PYTHON} ${HDF5_LIBRARIES} )

set_target_properties( _statismo_VTK PROPERTIES
  DEBUG_POSTFIX "-d"
  VERSION "${statismo_LIB_VERSION}"
  SOVERSION "${statismo_LIB_SOVERSION}"
)

install( TARGETS _statismo_VTK
  EXPORT statismoTargets
  RUNTIME DESTINATION ${INSTALL_BIN_DIR}
  LIBRARY DESTINATION ${INSTALL_LIB_DIR}
  ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
)
