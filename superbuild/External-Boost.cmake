#---------------------------------------------------------------------------
# Get and build boost

message( "External project - Boost" )

set( Boost_Bootstrap_Command )
if( UNIX )
  set( Boost_Bootstrap_Command ./bootstrap.sh )
  set( Boost_b2_Command ./b2 )
  set( SHELL_CMD sh)
else()
  if( WIN32 )
    set( Boost_Bootstrap_Command bootstrap.bat )
    set( Boost_b2_Command b2.exe )
  set (SHELL_CMD "")
  endif()
endif()

set( Boost_Patches_DIR ${Patches_DIR}/boost )
set( Boost_Patch_Script ${Boost_Patches_DIR}/boost_patch.sh )
set( Boost_Patch_Command ${SHELL_CMD} ${Boost_Patch_Script} )

ExternalProject_Add(Boost
  URL "http://sourceforge.net/projects/boost/files/boost/1.54.0/boost_1_54_0.tar.gz"
  URL_MD5 efbfbff5a85a9330951f243d0a46e4b9
  UPDATE_COMMAND ""
  PATCH_COMMAND ${Boost_Patch_Command}
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_BINARY_DIR}/Boost-prefix/src/Boost/boost
    ${INSTALL_DEPECENCIES_DIR}/include/boost
)

set(Boost_INCLUDE_DIR ${INSTALL_DEPECENCIES_DIR}/include )
