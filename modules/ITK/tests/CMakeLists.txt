set(TARGET_TESTS
  itkStandardImageRepresenterTest.cxx
  itkStandardMeshRepresenterTest.cxx
)

create_test_sourcelist( 
    itkStatismoTest
    itkStatismoTest_driver.cxx
    ${TARGET_TESTS}
)

add_executable(itkStatismoTest itkStatismoTest_driver.cxx ${TARGET_TESTS})
target_link_libraries(itkStatismoTest statismo_ITK)

foreach(TST ${TARGET_TESTS})
    get_filename_component(TNAME ${TST} NAME_WE)
    add_test(NAME ${TNAME} COMMAND itkStatismoTest ${TNAME} ${statismo_SOURCE_DIR}/data WORKING_DIRECTORY "${TEST_WORKING_DIR}")
endforeach()
