#---------------------------------------------------------------------------
# Get and build boost

message( "External project - Boost" )

set( Boost_Bootstrap_Command )

# Note: It IS important to download different files on different OS's:
# on Unix-like systems, we need the file persmissions (only available in the .tar.gz),
# while on Windows, we need CR/LF line feeds (only available in the .zip)

if( UNIX )
  set( Boost_url "http://sourceforge.net/projects/boost/files/boost/1.59.0/boost_1_59_0.tar.gz")
  set( Boost_md5 51528a0e3b33d9e10aaa311d9eb451e3 )
  set( Boost_Bootstrap_Command ./bootstrap.sh )
  set( Boost_b2_Command ./b2 )
else()
  if( WIN32 )
    set( Boost_url "http://sourceforge.net/projects/boost/files/boost/1.59.0/boost_1_59_0.zip")
    set( Boost_md5 08d29a2d85db3ebc8c6fdfa3a1f2b83c )
    set( Boost_Bootstrap_Command cmd /C bootstrap.bat msvc )
    set( Boost_b2_Command b2.exe )
  endif()
endif()

if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(Boost_address_model 64)
else()
  set(Boost_address_model 32)
endif()

if (MSVC_VERSION EQUAL 1500) #VS2008
set(boost_toolset "--toolset=msvc-9.0")
elseif(MSVC_VERSION EQUAL 1600) #VS2010
set(boost_toolset "--toolset=msvc-10.0")
elseif(MSVC_VERSION EQUAL 1700) #VS2012
set(boost_toolset "--toolset=msvc-11.0")
elseif(MSVC_VERSION EQUAL 1800) #VS2013
set(boost_toolset "--toolset=msvc-12.0")
elseif(MSVC_VERSION EQUAL 1900) #VS2015
set(boost_toolset "--toolset=msvc-14.0")
endif(MSVC_VERSION EQUAL 1500)


if(${BUILD_SHARED_LIBS} MATCHES OFF)
  set(BUILD_LIBS "static")
elseif(${BUILD_SHARED_LIBS} MATCHES ON)
  set(BUILD_LIBS "shared")
endif()

ExternalProject_Add(Boost
  BUILD_IN_SOURCE 1
  URL ${Boost_url}
  URL_MD5 ${Boost_md5}
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND ${Boost_Bootstrap_Command} --prefix=${INSTALL_DEPENDENCIES_DIR}/lib
  BUILD_COMMAND ${Boost_b2_Command} install -j8   --prefix=${INSTALL_DEPENDENCIES_DIR} --with-thread --with-filesystem --with-system --with-date_time --with-program_options  --with-atomic  address-model=${Boost_address_model} link=static,shared ${boost_toolset}
  INSTALL_COMMAND ""
)

if( WIN32 )
  set( Boost_INCLUDE_DIR ${INSTALL_DEPENDENCIES_DIR}/include/boost-1_59 )
  set( BOOST_ROOT ${INSTALL_DEPENDENCIES_DIR} )
else()
  set( Boost_INCLUDE_DIR ${INSTALL_DEPENDENCIES_DIR}/include )
endif()

set( Boost_LIBRARY_DIR ${INSTALL_DEPENDENCIES_DIR}/lib )
