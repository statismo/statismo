set( Eigen3_VERSION "3.2.0" )

ExternalProject_Add( Eigen3
  URL "http://bitbucket.org/eigen/eigen/get/${Eigen3_VERSION}.tar.gz"
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_BINARY_DIR}/Eigen3-prefix/src/Eigen3/Eigen
    ${INSTALL_DEPECENCIES_DIR}
)

set(EIGEN3_INCLUDE_DIR ${INSTALL_DEPECENCIES_DIR}/include )
