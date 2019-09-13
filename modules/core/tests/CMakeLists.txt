set(_target_tests
  basicStatismoTest.cxx
  utilsStatismoTest.cxx
)

create_test_sourcelist( 
    coreStatismoTest
    coreStatismoTest_driver.cxx
    ${_target_tests}
)

add_executable(coreStatismoTest coreStatismoTest_driver.cxx ${_target_tests})
target_link_libraries(coreStatismoTest statismo_core)

foreach(_tst ${_target_tests})
    get_filename_component(TNAME ${_tst} NAME_WE)
    add_test(NAME ${TNAME} COMMAND coreStatismoTest ${TNAME} WORKING_DIRECTORY "${TEST_WORKING_DIR}")
endforeach()