set( _CoreTests
  basicStatismoTest
)

foreach( _test ${_CoreTests} )
  add_executable( ${_test} ${_test}.cxx )
  target_link_libraries( ${_test} statismo_core ${ITK_LIBRARIES} )
endforeach()

add_test( NAME basicStatismoTest
  COMMAND basicStatismoTest
  WORKING_DIRECTORY "${TEST_WORKING_DIR}" )
