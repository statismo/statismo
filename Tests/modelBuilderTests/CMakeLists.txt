project(ModelBuilderTests)
cmake_minimum_required(VERSION 2.6)

enable_testing()
#include(Dart)

find_package(ITK 4.1 REQUIRED)
include (${ITK_USE_FILE})

find_package(VTK REQUIRED)
include (${VTK_USE_FILE})

string(REGEX REPLACE "itkhdf5_cpp" "" ITK_LIBRARIES "${ITK_LIBRARIES}")
string(REGEX REPLACE "itkhdf5" "" ITK_LIBRARIES "${ITK_LIBRARIES}")

include_directories(${STATISMO_ROOT_DIR} ${STATISMO_ROOT_DIR}/statismo_ITK
					${STATISMO_ROOT_DIR}/3rdParty  ${STATISMO_INCLUDE_DIR}
                  	${ITK_INCLUDE_DIRS}  ${STATISMO_ROOT_DIR}/Representers/ITK
					${STATISMO_ROOT_DIR}/Representers/VTK ${VTK_INCLUDE_DIR}
					${HDF5_INCLUDE_DIR} ${HDF5_INCLUDE_DIR_CPP})

link_directories(   ${HDF5_LIBRARY_DIR} ${ITK_LIBRARY_DIRS} ${STATISMO_LIBRARY_DIR} ${VTK_LIBRARY_DIR})


add_executable(PosteriorModelBuilderTest PosteriorModelBuilderTest.cpp)
add_dependencies(PosteriorModelBuilderTest HDF5)
target_link_libraries(PosteriorModelBuilderTest ${VTK_LIBRARIES} ${STATISMO_LIBRARIES} ${HDF5_LIBRARIES})

add_test(PosteriorModelBuilderTest  ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/PosteriorModelBuilderTest ${STATISMO_ROOT_DIR}/data)
