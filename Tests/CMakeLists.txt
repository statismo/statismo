include_directories(
  ${STATISMO_ROOT_DIR}
  ${STATISMO_ROOT_DIR}/Representers
  ${HDF5_INCLUDE_DIRS}
)

link_directories(${VTK_LIBRARY_DIR} ${HDF5_LIBRARY_DIRS})


add_executable(basicStatismoTest basicStatismoTest.cpp)
target_link_libraries(basicStatismoTest ${HDF5_LIBRARIES})

add_test(basicStatismoTest ${CMAKE_BINARY_DIR}/bin/basicStatismoTest)

add_subdirectory(modelBuilderTests)
add_subdirectory(representerTests)

