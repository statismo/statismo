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

set( _examples
  vtkBasicSamplingExample
  vtkBuildConditionalModelExample
  vtkBuildGaussianProcessShapeModelExample
  vtkBuildIntensityModelExample
  vtkBuildPosteriorModelExample
  vtkBuildShapeModelExample
  vtkCrossValidationExample
  vtkReduceModelVarianceExample
)

foreach( _e ${_examples} )
  add_executable( ${_e} ${_e}.cxx )
  target_link_libraries( ${_e} ${STATISMO_LIBRARIES} ${VTK_LIBRARIES} ${HDF5_LIBRARIES} )
endforeach()

install( TARGETS ${_examples}
  RUNTIME DESTINATION bin
)

install( PROGRAMS runVTKExamples.sh
  DESTINATION bin
)
