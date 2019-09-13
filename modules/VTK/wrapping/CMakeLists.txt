find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_PATH})

# The wrapper requires access to numpy headers (see numpy.i).
exec_program(${PYTHON_EXECUTABLE}
            ARGS "-c \"import numpy; print(numpy.get_include())\""
            OUTPUT_VARIABLE NUMPY_INCLUDE_DIR
            RETURN_VALUE NUMPY_NOT_FOUND
           )
set(NUMPY_INCLUDE_DIR ${NUMPY_INCLUDE_DIR} CACHE STRING "Numpy include path")
MARK_AS_ADVANCED( NUMPY_INCLUDE_DIR )
if(NUMPY_NOT_FOUND)
  message(FATAL_ERROR "Numpy package/header is missing" )
else()
  include_directories( "${NUMPY_INCLUDE_DIR}" )
endif()

# Find SWIG
find_package(SWIG REQUIRED)
include(${SWIG_USE_FILE})

# Prepare the wrapping library.
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${VTK_INCLUDE_DIRS})
set_source_files_properties(statismo.i PROPERTIES CPLUSPLUS ON)
set_source_files_properties(statismo.i PROPERTIES SWIG_FLAGS "-includeall")
file(GLOB TARGET_SOURCES ${statismo_SOURCE_DIR}/modules/VTK/src/*.cxx )

if("${CMAKE_VERSION}" VERSION_LESS "3.8.0")
  swig_add_module(statismo_VTK python statismo.i ${TARGET_SOURCES})
else()
  swig_add_library(statismo_VTK
                    MODULE LANGUAGE python
                    SOURCES statismo.i ${TARGET_SOURCES} )
endif()
swig_link_libraries(statismo_VTK statismo_core
                     ${PYTHON_LIBRARIES}
                     ${VTK_LIBRARIES}
                     ${HDF5_LIBRARIES})
target_link_libraries(_statismo_VTK PUBLIC ${HDF5_LIBRARIES})
set_target_properties(_statismo_VTK PROPERTIES
  DEBUG_POSTFIX "-d"
)

install(TARGETS _statismo_VTK
  EXPORT statismoTargets
  RUNTIME DESTINATION ${INSTALL_BIN_DIR}
  LIBRARY DESTINATION ${INSTALL_LIB_DIR}
  ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
)
