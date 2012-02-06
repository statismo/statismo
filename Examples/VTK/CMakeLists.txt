CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(ssm_vtk)
SET(CMAKE_VERBOSE_MAKEFILE ON)

FIND_PATH(STATISMO_DIR STATISMO REQUIRED)
include(${STATISMO_DIR}/statismo-config.cmake)

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

FILE(GLOB INCLUDE_FILES *.h)
FILE(GLOB SOURCE_FILES  *.txx *.cpp)

# TODO how can we get rid of the ../StatisticalModel include?
include_directories(${STATISMO_DIR} ${STATISMO_INCLUDE_DIR} ${VTK_INCLUDE_DIR}  ${STATISMO_DIR}/Representers/VTK)
link_directories(${STATISMO_LIBRARY_DIR} ${VTK_LIBRARY_DIR})
 
add_executable (BuildShapeModelExample BuildShapeModelExample.cxx) 
target_link_libraries (BuildShapeModelExample ${STATISMO_LIBRARIES} ${VTK_LIBRARIES})

add_executable (BasicSamplingExample BasicSamplingExample.cxx) 
target_link_libraries (BasicSamplingExample ${STATISMO_LIBRARIES} ${VTK_LIBRARIES})

add_executable (CrossValidationExample CrossValidationExample.cxx) 
target_link_libraries (CrossValidationExample ${STATISMO_LIBRARIES} ${VTK_LIBRARIES})

add_executable (BuildIntensityModelExample BuildIntensityModelExample.cxx) 
target_link_libraries (BuildIntensityModelExample ${STATISMO_LIBRARIES} ${VTK_LIBRARIES})

add_executable (BuildPartiallyFixedModelExample BuildPartiallyFixedModelExample.cxx) 
target_link_libraries (BuildPartiallyFixedModelExample ${STATISMO_LIBRARIES} ${VTK_LIBRARIES})

add_executable (BuildConditionalModelExample BuildConditionalModelExample.cxx) 
target_link_libraries (BuildConditionalModelExample ${STATISMO_LIBRARIES} ${VTK_LIBRARIES})
