cmake_minimum_required (VERSION 2.6)

SET(CMAKE_VERBOSE_MAKEFILE ON)
FIND_PATH(STATISMO_DIR STATISMO REQUIRED)
include(${STATISMO_DIR}/statismo-config.cmake)

FIND_PACKAGE(PythonLibs)
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})

FIND_PACKAGE(VTK REQUIRED)
include (${VTK_USE_FILE})

set(VTK_LIBRARIES
        vtkCommon 
        vtkFiltering 
        vtkGraphics 
        vtkImaging 
        vtkIO 
        vtkRendering 
        vtkWidgets 
)

set (VTK_LIBRARIES_PYTHON
        vtkCommonPythonD
        )

include_directories(${STATISMO_DIR} ${STATISMO_INCLUDE_DIR} ${VTK_INCLUDE_DIR}  ${STATISMO_DIR}/Representers/VTK)
link_directories (../StatisticalModel ${STATISMO_LIBRARY_DIR})   


FIND_PACKAGE(SWIG REQUIRED)
INCLUDE(${SWIG_USE_FILE})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

SET_SOURCE_FILES_PROPERTIES(statismo.i PROPERTIES CPLUSPLUS ON)
SET_SOURCE_FILES_PROPERTIES(statismo.i PROPERTIES SWIG_FLAGS "-includeall")

SWIG_ADD_MODULE(statismo python statismo.i  )
SWIG_LINK_LIBRARIES(statismo ${PYTHON_LIBRARIES}  ${STATISMO_LIBRARIES} ${VTK_LIBRARIES} ${VTK_LIBRARIES_PYTHON})


# # now for the java stuff
# FIND_PACKAGE(JNI REQUIRED)
# include_directories(${JNI_INCLUDE_DIRS})
# LIST(APPEND options -package statismoJava)
# SET_SOURCE_FILES_PROPERTIES(statismo.i PROPERTIES SWIG_FLAGS "${options}")
# SET(CMAKE_SWIG_OUTDIR ${CMAKE_CURRENT_BINARY_DIR}/statismoJava)

# SWIG_ADD_MODULE(statismoJava java  statismo.i)
# SWIG_LINK_LIBRARIES(statismoJava   ${STATISTICAL_MODEL_LIBRARIES} ${VTK_LIBRARIES})

# ADD_CUSTOM_COMMAND(TARGET statismoJava
# 				   POST_BUILD
# 				   COMMAND cmake -E echo "-----------------------------"
# 				   COMMAND cmake -E echo "Compiling Java files..."
# 				   COMMAND javac ${CMAKE_SWIG_OUTDIR}/*.java -d .
# 				   COMMAND cmake -E echo "-----------------------------"
# 				   COMMAND cmake -E echo "Creating jar file..."
# 				   COMMAND jar cvf statismoJava.jar ${CMAKE_SWIG_OUTDIR}
# 				   COMMAND cmake -E echo "-----------------------------"
# )
