set( _ITKTests
  itkStandardImageRepresenterTest
  itkStandardMeshRepresenterTest
)

foreach( _test ${_ITKTests} )
  add_executable( ${_test} ${_test}.cxx )
  target_link_libraries( ${_test} statismo-core ${ITK_LIBRARIES} )
endforeach()



