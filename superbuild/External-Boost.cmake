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
    set( Boost_Bootstrap_Command cmd /C bootstrap.bat msvc )
    set( Boost_b2_Command b2.exe )
    set (SHELL_CMD "")
  endif()
endif()

set( Boost_Patches_DIR ${Patches_DIR}/boost )
set( Boost_Patch_Script ${Boost_Patches_DIR}/boost_patch.sh )
set( Boost_Patch_Command ${SHELL_CMD} ${Boost_Patch_Script} )


ExternalProject_Add(Boost
  BUILD_IN_SOURCE 1
  URL "http://sourceforge.net/projects/boost/files/boost/1.55.0/boost_1_55_0.tar.gz"
  URL_MD5 93780777cfbf999a600f62883bd54b17
  UPDATE_COMMAND ""
  PATCH_COMMAND ${Boost_Patch_Command}
  CONFIGURE_COMMAND ${Boost_Bootstrap_Command} --prefix=${INSTALL_DEPECENCIES_DIR}/lib --with-libraries=thread --with-libraries=system
  BUILD_COMMAND ${Boost_b2_Command} install -j8   --prefix=${INSTALL_DEPECENCIES_DIR}
  INSTALL_COMMAND ""
)

if( WIN32 )
  set( Boost_INCLUDE_DIR ${INSTALL_DEPECENCIES_DIR}/include/boost-1_54 )
  set( BOOST_ROOT ${INSTALL_DEPECENCIES_DIR} )
else()
  set( Boost_INCLUDE_DIR ${INSTALL_DEPECENCIES_DIR}/include )
endif()

set( Boost_LIBRARY_DIR ${INSTALL_DEPECENCIES_DIR}/lib )
