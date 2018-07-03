message( "External project - Eigen" )

ExternalProject_Add( Eigen3
  URL "http://bitbucket.org/eigen/eigen/get/3.3.4.tar.gz"
  URL_HASH "SHA256=4286e8f1fabbd74f7ec6ef8ef31c9dbc6102b9248a8f8327b04f5b68da5b05e1"
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND
    ${CMAKE_COMMAND} -E copy_directory
      ${CMAKE_BINARY_DIR}/Eigen3-prefix/src/Eigen3/Eigen
      ${INSTALL_DEPENDENCIES_DIR}/include/Eigen3/Eigen &&
    ${CMAKE_COMMAND} -E copy_directory
      ${CMAKE_BINARY_DIR}/Eigen3-prefix/src/Eigen3/unsupported
      ${INSTALL_DEPENDENCIES_DIR}/include/Eigen3/unsupported
)

set(EIGEN3_INCLUDE_DIR ${INSTALL_DEPENDENCIES_DIR}/include/Eigen3 )
