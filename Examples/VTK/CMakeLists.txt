CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(ssm_vtk)
SET(CMAKE_VERBOSE_MAKEFILE ON)

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


include_directories(${STATISMO_ROOT_DIR}  ${STATISMO_ROOT_DIR}/3rdParty  ${STATISMO_INCLUDE_DIR} 
					${VTK_INCLUDE_DIR}  ${STATISMO_ROOT_DIR}/Representers/VTK 
					${HDF5_INCLUDE_DIR} ${HDF5_INCLUDE_DIR_CPP})					

link_directories(${STATISMO_LIBRARY_DIR} ${VTK_LIBRARY_DIR} ${HDF5_LIBRARY_DIR})
 
add_executable (BuildShapeModelExample BuildShapeModelExample.cxx) 
target_link_libraries (BuildShapeModelExample ${STATISMO_LIBRARIES} ${VTK_LIBRARIES} ${HDF5_LIBRARIES})

add_executable (BasicSamplingExample BasicSamplingExample.cxx) 
target_link_libraries (BasicSamplingExample ${STATISMO_LIBRARIES} ${VTK_LIBRARIES} ${HDF5_LIBRARIES})

add_executable (CrossValidationExample CrossValidationExample.cxx) 
target_link_libraries (CrossValidationExample ${STATISMO_LIBRARIES} ${VTK_LIBRARIES} ${HDF5_LIBRARIES})

add_executable (BuildIntensityModelExample BuildIntensityModelExample.cxx) 
target_link_libraries (BuildIntensityModelExample ${STATISMO_LIBRARIES} ${VTK_LIBRARIES} ${HDF5_LIBRARIES})

add_executable (BuildPartiallyFixedModelExample BuildPartiallyFixedModelExample.cxx) 
target_link_libraries (BuildPartiallyFixedModelExample ${STATISMO_LIBRARIES} ${VTK_LIBRARIES} ${HDF5_LIBRARIES})

add_executable (BuildConditionalModelExample BuildConditionalModelExample.cxx) 
target_link_libraries (BuildConditionalModelExample ${STATISMO_LIBRARIES} ${VTK_LIBRARIES} ${HDF5_LIBRARIES})
