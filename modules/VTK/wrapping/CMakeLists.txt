find_package( PythonLibs REQUIRED )
include_directories(${PYTHON_INCLUDE_PATH})

# VTK material for linking.
# See also the list of required modules in ../CMakeLists.txt
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

# The wrapper requires access to numpy headers (see numpy.i).
exec_program( ${PYTHON_EXECUTABLE}
            ARGS "-c \"import numpy; print(numpy.get_include())\""
            OUTPUT_VARIABLE NUMPY_INCLUDE_DIR
            RETURN_VALUE NUMPY_NOT_FOUND
           )
set( NUMPY_INCLUDE_DIR ${NUMPY_INCLUDE_DIR} CACHE STRING "Numpy include path" )
MARK_AS_ADVANCED( NUMPY_INCLUDE_DIR )
if( NUMPY_NOT_FOUND )
message( FATAL_ERROR "Numpy package/header is missing" )
else()
include_directories( "${NUMPY_INCLUDE_DIR}" )
endif()

# Find SWIG
find_package( SWIG REQUIRED )
include( ${SWIG_USE_FILE} )

# Prepare the wrapping library.
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${VTK_INCLUDE_DIRS})
set_source_files_properties(statismo.i PROPERTIES CPLUSPLUS ON)
set_source_files_properties(statismo.i PROPERTIES SWIG_FLAGS "-includeall")
file( GLOB _Statismo_src ${statismo_SOURCE_DIR}/modules/VTK/src/*.cxx )
if( "${CMAKE_VERSION}" VERSION_LESS "3.8.0" )
  swig_add_module( statismo_VTK python statismo.i ${_Statismo_src} )
else()
  swig_add_library( statismo_VTK
                    MODULE LANGUAGE python
                    SOURCES statismo.i ${_Statismo_src} )
endif()
swig_link_libraries( statismo_VTK statismo_core
                     ${PYTHON_LIBRARIES}
                     ${VTK_LIBRARIES} ${VTK_LIBRARIES_PYTHON}
                     ${HDF5_LIBRARIES} )

set_target_properties( _statismo_VTK PROPERTIES
  DEBUG_POSTFIX "-d"
)

install( TARGETS _statismo_VTK
  EXPORT statismoTargets
  RUNTIME DESTINATION ${INSTALL_BIN_DIR}
  LIBRARY DESTINATION ${INSTALL_LIB_DIR}
  ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
)
