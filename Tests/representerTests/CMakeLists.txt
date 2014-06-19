PROJECT(RepresenterTests)
cmake_minimum_required(VERSION 2.6)


#
# Note! As we are depending on ITK anyway, we use for all the tests the hdf5 libraries that are shipped with ITK (even for the VTK examples).
#

ENABLE_TESTING()

FIND_PACKAGE(ITK 4.1 REQUIRED)
include (${ITK_USE_FILE})

FIND_PACKAGE(VTK REQUIRED)
include (${VTK_USE_FILE})

# HACK, we get the main include path for the system (which we hope is the head of the itk_include_dirs variable)
# Then we augment it with itkhdf5 to get access to itk's hdf5 includes
list(GET ITK_INCLUDE_DIRS 0 ITK_MAIN_INCLUDE_DIR)
SET (ITK_HDF5_INCLUDE_DIR ${ITK_MAIN_INCLUDE_DIR}/itkhdf5 ${ITK_MAIN_INCLUDE_DIR}/itkhdf5/cpp)

include_directories(${ITK_HDF5_INCLUDE_DIR}
                    ${ITK_INCLUDE_DIRS}
                    ${STATISMO_ROOT_DIR}
                    ${STATISMO_ROOT_DIR}/statismo_ITK
                    ${STATISMO_INCLUDE_DIR}
                    ${STATISMO_ROOT_DIR}/Representers/ITK
                    ${STATISMO_ROOT_DIR}/Representers/VTK
                    ${STATISMO_ROOT_DIR}/3rdParty
                    ${VTK_INCLUDE_DIR} )

link_directories( ${ITK_LIBRARY_DIRS} ${VTK_LIBRARY_DIR})



ADD_EXECUTABLE(itkStandardImageRepresenterTest itkStandardImageRepresenterTest.cpp)
ADD_DEPENDENCIES(itkStandardImageRepresenterTest HDF5)
TARGET_LINK_LIBRARIES(itkStandardImageRepresenterTest ${ITK_LIBRARIES} ${STATISMO_LIBRARIES})

ADD_EXECUTABLE(itkStandardMeshRepresenterTest itkStandardMeshRepresenterTest.cpp)
ADD_DEPENDENCIES(itkStandardMeshRepresenterTest HDF5)
TARGET_LINK_LIBRARIES(itkStandardMeshRepresenterTest   ${ITK_LIBRARIES} )


ADD_EXECUTABLE(vtkStandardMeshRepresenterTest vtkStandardMeshRepresenterTest.cpp)
ADD_DEPENDENCIES(vtkStandardMeshRepresenterTest HDF5)
TARGET_LINK_LIBRARIES(vtkStandardMeshRepresenterTest ${VTK_LIBRARIES}  ${ITK_LIBRARIES})

ADD_EXECUTABLE(vtkStandardImageRepresenterTest vtkStandardImageRepresenterTest.cpp)
ADD_DEPENDENCIES(vtkStandardImageRepresenterTest HDF5)
TARGET_LINK_LIBRARIES(vtkStandardImageRepresenterTest ${VTK_LIBRARIES}  ${ITK_LIBRARIES})


ADD_TEST(itkStandardImageRepresenterTest  ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/itkStandardImageRepresenterTest ${STATISMO_ROOT_DIR}/data)
ADD_TEST(itkStandardMeshRepresenterTest  ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/itkStandardMeshRepresenterTest ${STATISMO_ROOT_DIR}/data)
ADD_TEST(vtkStandardMeshRepresenterTest  ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/vtkStandardMeshRepresenterTest ${STATISMO_ROOT_DIR}/data)
ADD_TEST(vtkStandardImageRepresenterTest ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/vtkStandardImageRepresenterTest ${STATISMO_ROOT_DIR}/data)
