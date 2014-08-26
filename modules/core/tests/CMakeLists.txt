set( _CoreTests
  basicStatismoTest
)

foreach( _test ${_CoreTests} )
  add_executable( ${_test} ${_test}.cxx )
  target_link_libraries( ${_test} statismo-core )
endforeach()
