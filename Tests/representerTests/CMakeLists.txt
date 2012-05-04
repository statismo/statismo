PROJECT(RepresenterTests)
cmake_minimum_required(VERSION 2.6)

ENABLE_TESTING()
#INCLUDE(Dart)

FIND_PACKAGE(ITK 4.1 REQUIRED)
include (${ITK_USE_FILE})

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

FIND_PACKAGE(STATISMO REQUIRED)

include_directories(${STATISMO_ROOT_DIR} ${STATISMO_ROOT_DIR}/statismo_ITK  
					${STATISMO_ROOT_DIR}/3rdParty  ${STATISMO_INCLUDE_DIR}  
					${ITK_INCLUDE_DIRS}  ${STATISMO_ROOT_DIR}/Representers/ITK 
					${STATISMO_ROOT_DIR}/Representers/VTK ${VTK_INCLUDE_DIR} )

link_directories(${ITK_LIBRARY_DIRS} ${STATISMO_LIBRARY_DIR} ${VTK_LIBRARY_DIR})



ADD_EXECUTABLE(itkVectorImageRepresenterTest itkVectorImageRepresenterTest.cpp) 
TARGET_LINK_LIBRARIES(itkVectorImageRepresenterTest ${ITK_LIBRARIES} ${STATISMO_LIBRARIES})


ADD_EXECUTABLE(vtkPolyDataRepresenterTest vtkPolyDataRepresenterTest.cpp) 
TARGET_LINK_LIBRARIES(vtkPolyDataRepresenterTest ${VTK_LIBRARIES} ${STATISMO_LIBRARIES})


ADD_TEST(itkVectorImageRepresenterTest ${CMAKE_CURRENT_BINARY_DIR}/itkVectorImageRepresenterTest ${STATISMO_ROOT_DIR}/data)
ADD_TEST(vtkPolyDataRepresenterTest ${CMAKE_CURRENT_BINARY_DIR}/vtkPolyDataRepresenterTest ${STATISMO_ROOT_DIR}/data)

