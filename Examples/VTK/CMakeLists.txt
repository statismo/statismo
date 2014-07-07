cmake_minimum_required(VERSION 2.6)
project(ssm_vtk)

file(GLOB INCLUDE_FILES *.h)
file(GLOB SOURCE_FILES  *.txx *.cpp)


include_directories(${STATISMO_ROOT_DIR}
					${STATISMO_ROOT_DIR}/Representers/VTK
					${HDF5_INCLUDE_DIRS}
  )

link_directories(${STATISMO_LIBRARY_DIR} ${VTK_LIBRARY_DIR} ${HDF5_LIBRARY_DIRS})

if(WIN32)
  # Fix for VS2012 that has _VARIADIC_MAX set to 5. For some of the examples, this does not seem to be sufficient.
  #Don't set too high because it increases compiler memory usage / compile-time.
  add_definitions( -D_VARIADIC_MAX=9 )
endif()

add_executable (vtkBuildShapeModelExample BuildShapeModelExample.cxx)
target_link_libraries (vtkBuildShapeModelExample ${STATISMO_LIBRARIES} ${VTK_LIBRARIES} )# ${HDF5_LIBRARIES})

add_executable (vtkBasicSamplingExample BasicSamplingExample.cxx)
target_link_libraries (vtkBasicSamplingExample ${VTK_LIBRARIES} ${HDF5_LIBRARIES})

add_executable (vtkCrossValidationExample CrossValidationExample.cxx)
target_link_libraries (vtkCrossValidationExample ${VTK_LIBRARIES} ${HDF5_LIBRARIES})

add_executable (vtkBuildIntensityModelExample BuildIntensityModelExample.cxx)
target_link_libraries (vtkBuildIntensityModelExample ${VTK_LIBRARIES} ${HDF5_LIBRARIES})

add_executable (vtkReduceModelVarianceExample ReduceModelVarianceExample.cxx)
target_link_libraries (vtkReduceModelVarianceExample ${VTK_LIBRARIES} ${HDF5_LIBRARIES})


add_executable (vtkBuildPosteriorModelExample BuildPosteriorModelExample.cxx)
target_link_libraries (vtkBuildPosteriorModelExample ${VTK_LIBRARIES} ${HDF5_LIBRARIES})


add_executable (vtkBuildConditionalModelExample BuildConditionalModelExample.cxx)
target_link_libraries (vtkBuildConditionalModelExample  ${VTK_LIBRARIES} ${HDF5_LIBRARIES})

add_executable (vtkBuildGaussianProcessShapeModelExample BuildGaussianProcessShapeModelExample.cxx)
target_link_libraries (vtkBuildGaussianProcessShapeModelExample ${STATISMO_LIBRARIES} ${VTK_LIBRARIES} ${HDF5_LIBRARIES})

install(TARGETS
vtkBuildShapeModelExample
vtkBasicSamplingExample
vtkCrossValidationExample
vtkBuildIntensityModelExample
vtkBuildPosteriorModelExample
vtkBuildConditionalModelExample
vtkBuildGaussianProcessShapeModelExample
RUNTIME DESTINATION bin)

install(PROGRAMS runVTKExamples.sh DESTINATION  bin)
