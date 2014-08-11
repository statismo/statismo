find_package(ITK REQUIRED)
include (${ITK_USE_FILE})

find_package(VTK REQUIRED)
include (${VTK_USE_FILE})

string(REGEX REPLACE "itkhdf5_cpp" "" ITK_LIBRARIES "${ITK_LIBRARIES}")
string(REGEX REPLACE "itkhdf5" "" ITK_LIBRARIES "${ITK_LIBRARIES}")

include_directories(
  ${STATISMO_ROOT_DIR}
  ${STATISMO_ROOT_DIR}/statismo_ITK
  ${STATISMO_ROOT_DIR}/3rdParty
  ${STATISMO_INCLUDE_DIR}
  ${ITK_INCLUDE_DIRS}
  ${STATISMO_ROOT_DIR}/Representers/ITK
  ${STATISMO_ROOT_DIR}/Representers/VTK
  ${VTK_INCLUDE_DIR}
  ${HDF5_INCLUDE_DIR}
  ${HDF5_INCLUDE_DIR_CPP}
)

add_executable(PosteriorModelBuilderTest PosteriorModelBuilderTest.cpp)
target_link_libraries(PosteriorModelBuilderTest ${VTK_LIBRARIES} ${STATISMO_LIBRARIES} ${HDF5_LIBRARIES})

add_test(PosteriorModelBuilderTest  ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/PosteriorModelBuilderTest ${STATISMO_ROOT_DIR}/data)
