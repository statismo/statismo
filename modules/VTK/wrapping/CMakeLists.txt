find_package( PythonLibs REQUIRED )
include_directories(${PYTHON_INCLUDE_PATH})

find_package( VTK REQUIRED )
include( ${VTK_USE_FILE} )

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

swig_add_module( statismo python statismo.i ${_Statismo_src})
add_dependencies( _statismo HDF5 )

swig_link_libraries( statismo ${PYTHON_LIBRARIES}  statismo-core ${VTK_LIBRARIES} ${VTK_LIBRARIES_PYTHON} ${HDF5_LIBRARIES} )

install( TARGETS _statismo
  LIBRARY DESTINATION lib/Python
)
install( FILES ${CMAKE_CURRENT_BINARY_DIR}/statismo.py
  DESTINATION lib/Python
)
