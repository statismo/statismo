#---------------------------------------------------------------------------
# Get and build boost

message( "External project - Boost" )

# Note: It IS important to download different files on different OS's:
# on Unix-like systems, we need the file persmissions (only available in the .tar.gz),
# while on Windows, we need CR/LF line feeds (only available in the .zip)
if( WIN32 )
  set( Boost_url "http://dl.bintray.com/boostorg/release/1.65.1/source/boost_1_65_1.zip" )
  set( Boost_hash "SHA256=d1775aef807bd6a14077b450cce2950d8eacd86aaf51015f79e712917f8cc3c2" )
  set( Boost_Bootstrap_Command cmd /C bootstrap.bat msvc )
  set( Boost_b2_Command b2.exe )
  set( Boost_b2_extra_opts "runtime-link=shared" "threading=multi" "--build-type=complete" )
else()
  set( Boost_url "http://dl.bintray.com/boostorg/release/1.65.1/source/boost_1_65_1.tar.gz" )
  set( Boost_hash "SHA256=a13de2c8fbad635e6ba9c8f8714a0e6b4264b60a29b964b940a22554705b6b60" )
  set( Boost_Bootstrap_Command ./bootstrap.sh )
  set( Boost_b2_Command ./b2 )
  set( Boost_b2_extra_opts )
endif()
#make sure you update this when changing the boost version above
set( Boost_dir_postfix "1_65_1")


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
elseif(MSVC_VERSION GREATER 1909 AND MSVC_VERSION LESS 1920) #VS2017
set(boost_toolset "--toolset=msvc-14.1") #VS version says 15.0, but the toolset version is 14.1
endif(MSVC_VERSION EQUAL 1500)

if(${BUILD_SHARED_LIBS} MATCHES OFF OR WIN32)
  set(BUILD_LIBS "static")
else()
  set(BUILD_LIBS "shared")
endif()

ExternalProject_Add(Boost
  BUILD_IN_SOURCE 1
  URL ${Boost_url}
  URL_HASH ${Boost_hash}
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND ${Boost_Bootstrap_Command} --prefix=${INSTALL_DEPENDENCIES_DIR}/lib
  BUILD_COMMAND ${Boost_b2_Command} install -j8 --prefix=${INSTALL_DEPENDENCIES_DIR} --with-thread --with-filesystem --with-system --with-date_time --with-program_options  --with-atomic address-model=${Boost_address_model} link=${BUILD_LIBS} ${Boost_b2_extra_opts} ${boost_toolset}
  INSTALL_COMMAND ""
)

if( WIN32 )
  set( Boost_INCLUDE_DIR ${INSTALL_DEPENDENCIES_DIR}/include/boost-${Boost_dir_postfix} )
  set( BOOST_ROOT ${INSTALL_DEPENDENCIES_DIR} )
else()
  set( Boost_INCLUDE_DIR ${INSTALL_DEPENDENCIES_DIR}/include )
endif()

set( Boost_LIBRARY_DIR ${INSTALL_DEPENDENCIES_DIR}/lib )
