project(statismoTests)
cmake_minimum_required(VERSION 2.6)

#include(Dart)

include_directories(${STATISMO_ROOT_DIR} ${STATISMO_ROOT_DIR}/Representers
					${STATISMO_ROOT_DIR}/3rdParty ${HDF5_INCLUDE_DIR} ${HDF5_INCLUDE_DIR_CPP})

link_directories( ${HDF5_LIBRARY_DIR})

add_executable(basicStatismoTest basicStatismoTest.cpp)
add_dependencies(basicStatismoTest HDF5)
target_link_libraries(basicStatismoTest ${HDF5_LIBRARIES})
add_test(basicStatismoTest ${CMAKE_BINARY_DIR}/bin/basicStatismoTest)

add_subdirectory(modelBuilderTests)

