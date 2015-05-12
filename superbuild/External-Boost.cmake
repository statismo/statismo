#---------------------------------------------------------------------------
# Get and build boost

message( "External project - Boost" )

set( Boost_Bootstrap_Command )

# Note: It IS important to download different files on different OS's:
# on Unix-like systems, we need the file persmissions (only available in the .tar.gz),
# while on Windows, we need CR/LF line feeds (only available in the .zip)

if( UNIX )
  set( Boost_url "http://sourceforge.net/projects/boost/files/boost/1.56.0/boost_1_56_0.tar.gz")
  set( Boost_md5 8c54705c424513fa2be0042696a3a162 )
  set( Boost_Bootstrap_Command ./bootstrap.sh )
  set( Boost_b2_Command ./b2 )
else()
  if( WIN32 )
    set( Boost_url "http://sourceforge.net/projects/boost/files/boost/1.56.0/boost_1_56_0.zip")
    set( Boost_md5 fc16da21d641e03715a4144cb093964e )
    set( Boost_Bootstrap_Command cmd /C bootstrap.bat msvc )
    set( Boost_b2_Command b2.exe )
  endif()
endif()

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(Boost_address_model 64)
else()
  set(Boost_address_model 32)
endif()

ExternalProject_Add(Boost
  BUILD_IN_SOURCE 1
  URL ${Boost_url}
  URL_MD5 ${Boost_md5}
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND ${Boost_Bootstrap_Command} --prefix=${INSTALL_DEPENDENCIES_DIR}/lib
  BUILD_COMMAND ${Boost_b2_Command} install -j8   --prefix=${INSTALL_DEPENDENCIES_DIR} --with-thread --with-filesystem --with-system --with-date_time --with-program_options address-model=${Boost_address_model}
  INSTALL_COMMAND ""
)

if( WIN32 )
  set( Boost_INCLUDE_DIR ${INSTALL_DEPENDENCIES_DIR}/include/boost-1_56 )
  set( BOOST_ROOT ${INSTALL_DEPENDENCIES_DIR} )
else()
  set( Boost_INCLUDE_DIR ${INSTALL_DEPENDENCIES_DIR}/include )
endif()

set( Boost_LIBRARY_DIR ${INSTALL_DEPENDENCIES_DIR}/lib )
