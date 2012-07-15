PROJECT(statismoTests)
cmake_minimum_required(VERSION 2.6)


#INCLUDE(Dart)


include_directories(${STATISMO_ROOT_DIR} ${STATISMO_ROOT_DIR}/Representers  
					${STATISMO_ROOT_DIR}/3rdParty ${HDF5_INCLUDE_DIR} ${HDF5_INCLUDE_DIR_CPP})

link_directories(   ${HDF5_LIBRARY_DIR})



ADD_EXECUTABLE(basicStatismoTest basicStatismoTest.cpp) 
ADD_DEPENDENCIES(basicStatismoTest HDF5)
TARGET_LINK_LIBRARIES(basicStatismoTest ${HDF5_LIBRARIES})
ADD_TEST(basicStatismoTest ${CMAKE_BINARY_DIR}/bin/basicStatismoTest)
