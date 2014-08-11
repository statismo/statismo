cmake_minimum_required (VERSION 2.6)

set(CMAKE_VERBOSE_MAKEFILE ON)

find_package(PythonLibs)
include_directories(${PYTHON_INCLUDE_PATH})

find_package(VTK REQUIRED)
include (${VTK_USE_FILE})


if(VTK_MAJOR_VERSION EQUAL 5)
    # VTK 5 needs additional libraries
    set (VTK_LIBRARIES_PYTHON
        vtkCommonPythonD
        vtkPythonCore
        )
else(VTK_MAJOR_VERSION EQUAL 5)
    set (VTK_LIBRARIES_PYTHON "")
endif(VTK_MAJOR_VERSION EQUAL 5)

include_directories(${STATISMO_ROOT_DIR} ${STATISMO_INCLUDE_DIR} ${STATISMO_ROOT_DIR}/3rdParty 
					${VTK_INCLUDE_DIR}  ${STATISMO_ROOT_DIR}/Representers/VTK ${STATISMO_ROOT_DIR}/Representers
						${HDF5_INCLUDE_DIR} ${HDF5_INCLUDE_DIR_CPP})
						
link_directories (${STATISMO_LIBRARY_DIR} ${HDF5_LIBRARY_DIR})   


find_package(SWIG REQUIRED)
include(${SWIG_USE_FILE})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

set_source_files_properties(statismo.i PROPERTIES CPLUSPLUS ON)
set_source_files_properties(statismo.i PROPERTIES SWIG_FLAGS "-includeall")

swig_add_module(statismo python statismo.i  )
add_dependencies(_statismo HDF5)
swig_link_libraries(statismo ${PYTHON_LIBRARIES}  ${STATISMO_LIBRARIES} ${VTK_LIBRARIES} ${VTK_LIBRARIES_PYTHON} ${HDF5_LIBRARIES})

install(TARGETS _statismo  LIBRARY DESTINATION lib/Python)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/statismo.py DESTINATION lib/Python)
