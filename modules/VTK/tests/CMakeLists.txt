set( _VTKTests
  PosteriorModelBuilderTest
  vtkStandardImageRepresenterTest
  vtkStandardMeshRepresenterTest
)

foreach( _test ${_VTKTests} )
  add_executable( ${_test} ${_test}.cxx )
  target_link_libraries( ${_test} statismo-VTK )
endforeach()



