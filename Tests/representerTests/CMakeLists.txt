project(RepresenterTests)
cmake_minimum_required(VERSION 2.6)

#
# Note! As we are depending on ITK anyway, we use for all the tests the hdf5 libraries that are shipped with ITK (even for the VTK examples).
#

enable_testing()

find_package(ITK 4.1 REQUIRED)
include (${ITK_USE_FILE})

find_package(VTK REQUIRED)
include (${VTK_USE_FILE})

# HACK, we get the main include path for the system (which we hope is the head of the itk_include_dirs variable)
# Then we augment it with itkhdf5 to get access to itk's hdf5 includes
list(GET ITK_INCLUDE_DIRS 0 ITK_MAIN_INCLUDE_DIR)
set (ITK_HDF5_INCLUDE_DIR ${ITK_MAIN_INCLUDE_DIR}/itkhdf5 ${ITK_MAIN_INCLUDE_DIR}/itkhdf5/cpp)

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



add_executable(itkStandardImageRepresenterTest itkStandardImageRepresenterTest.cpp)
add_dependencies(itkStandardImageRepresenterTest HDF5)
target_link_libraries(itkStandardImageRepresenterTest ${ITK_LIBRARIES} ${STATISMO_LIBRARIES})

add_executable(itkStandardMeshRepresenterTest itkStandardMeshRepresenterTest.cpp)
add_dependencies(itkStandardMeshRepresenterTest HDF5)
target_link_libraries(itkStandardMeshRepresenterTest   ${ITK_LIBRARIES} )


add_executable(vtkStandardMeshRepresenterTest vtkStandardMeshRepresenterTest.cpp)
add_dependencies(vtkStandardMeshRepresenterTest HDF5)
target_link_libraries(vtkStandardMeshRepresenterTest ${VTK_LIBRARIES}  ${ITK_LIBRARIES})

add_executable(vtkStandardImageRepresenterTest vtkStandardImageRepresenterTest.cpp)
add_dependencies(vtkStandardImageRepresenterTest HDF5)
target_link_libraries(vtkStandardImageRepresenterTest ${VTK_LIBRARIES}  ${ITK_LIBRARIES})


add_test(itkStandardImageRepresenterTest  ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/itkStandardImageRepresenterTest ${STATISMO_ROOT_DIR}/data)
add_test(itkStandardMeshRepresenterTest  ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/itkStandardMeshRepresenterTest ${STATISMO_ROOT_DIR}/data)
add_test(vtkStandardMeshRepresenterTest  ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/vtkStandardMeshRepresenterTest ${STATISMO_ROOT_DIR}/data)
add_test(vtkStandardImageRepresenterTest ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/vtkStandardImageRepresenterTest ${STATISMO_ROOT_DIR}/data)
