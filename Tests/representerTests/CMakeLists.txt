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


include_directories(${STATISMO_ROOT_DIR} ${STATISMO_ROOT_DIR}/statismo_ITK  
					${STATISMO_ROOT_DIR}/3rdParty  ${STATISMO_INCLUDE_DIR}  
                  	${ITK_INCLUDE_DIRS}  ${STATISMO_ROOT_DIR}/Representers/ITK 
					${STATISMO_ROOT_DIR}/Representers/VTK ${VTK_INCLUDE_DIR} 
					${HDF5_INCLUDE_DIR} ${HDF5_INCLUDE_DIR_CPP})

link_directories(   ${HDF5_LIBRARY_DIR} ${ITK_LIBRARY_DIRS} ${STATISMO_LIBRARY_DIR} ${VTK_LIBRARY_DIR})



ADD_EXECUTABLE(itkVectorImageRepresenterTest itkVectorImageRepresenterTest.cpp) 
ADD_DEPENDENCIES(itkVectorImageRepresenterTest HDF5)
TARGET_LINK_LIBRARIES(itkVectorImageRepresenterTest ${ITK_LIBRARIES} ${STATISMO_LIBRARIES})

ADD_EXECUTABLE(itkImageRepresenterTest itkImageRepresenterTest.cpp) 
ADD_DEPENDENCIES(itkImageRepresenterTest HDF5)
TARGET_LINK_LIBRARIES(itkImageRepresenterTest   ${ITK_LIBRARIES} ${STATISMO_LIBRARIES})


ADD_EXECUTABLE(itkMeshRepresenterTest itkMeshRepresenterTest.cpp) 
ADD_DEPENDENCIES(itkMeshRepresenterTest HDF5)
TARGET_LINK_LIBRARIES(itkMeshRepresenterTest   ${ITK_LIBRARIES} ${STATISMO_LIBRARIES})


ADD_EXECUTABLE(vtkPolyDataRepresenterTest vtkPolyDataRepresenterTest.cpp) 
ADD_DEPENDENCIES(vtkPolyDataRepresenterTest HDF5)
TARGET_LINK_LIBRARIES(vtkPolyDataRepresenterTest ${VTK_LIBRARIES} ${STATISMO_LIBRARIES} ${HDF5_LIBRARIES})

ADD_EXECUTABLE(vtkStructuredPointsRepresenterTest vtkStructuredPointsRepresenterTest.cpp) 
ADD_DEPENDENCIES(vtkStructuredPointsRepresenterTest HDF5)
TARGET_LINK_LIBRARIES(vtkStructuredPointsRepresenterTest ${VTK_LIBRARIES} ${STATISMO_LIBRARIES}  ${HDF5_LIBRARIES})


ADD_TEST(itkVectorImageRepresenterTest ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/itkVectorImageRepresenterTest ${STATISMO_ROOT_DIR}/data)
ADD_TEST(itkImageRepresenterTest  ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/itkImageRepresenterTest ${STATISMO_ROOT_DIR}/data)
ADD_TEST(itkMeshRepresenterTest  ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/itkMeshRepresenterTest ${STATISMO_ROOT_DIR}/data)
ADD_TEST(vtkPolyDataRepresenterTest  ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/vtkPolyDataRepresenterTest ${STATISMO_ROOT_DIR}/data)
ADD_TEST(vtkStructuredPointsRepresenterTest ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/vtkStructuredPointsRepresenterTest ${STATISMO_ROOT_DIR}/data)
