set(TARGET_TESTS
  PosteriorModelBuilderTest.cxx
  PCAModelBuilderWithSelfAdjointEigenSolverTest.cxx
  PCAModelBuilderTest.cxx
  vtkStandardImageRepresenterTest.cxx
  vtkStandardMeshRepresenterTest.cxx
)

create_test_sourcelist( 
    vtkStatismoTest
    vtkStatismoTest_driver.cxx
    ${TARGET_TESTS}
)

add_executable(vtkStatismoTest vtkStatismoTest_driver.cxx ${TARGET_TESTS} vtkTestHelper.h)
target_link_libraries(vtkStatismoTest statismo_VTK)

foreach( _test ${_VTKTests} )
  add_executable( ${_test} ${_test}.cxx )
  target_link_libraries( ${_test} statismo_VTK )
endforeach()

foreach(TST ${TARGET_TESTS})
    get_filename_component(TNAME ${TST} NAME_WE)
    add_test(NAME ${TNAME} COMMAND vtkStatismoTest ${TNAME} ${statismo_SOURCE_DIR}/data WORKING_DIRECTORY "${TEST_WORKING_DIR}")
endforeach()
