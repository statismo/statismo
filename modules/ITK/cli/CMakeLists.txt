if(MSVC11) #i.e. Visual Studio 2012
	# Fix for VS2012 that has _VARIADIC_MAX set to 10. Don't set too high because it increases compiler memory usage / compile-time.
	add_definitions( -D_VARIADIC_MAX=10 )
	# Fix for another VS2012 problem: not all TR1 options are automatically detected, therefore we force them here.
	add_definitions( -D BOOST_HAS_TR1 )
	add_definitions( -D BOOST_NO_0X_HDR_INITIALIZER_LIST )
endif()

#the idea here is to have an X.cxx and a X.md where X denotes the cli-command name.
set( _cli_files
	statismo-build-shape-model
)

#search for pandoc to generate man pages or chm help files
find_program(PANDOC NAMES pandoc)
mark_as_advanced(PANDOC)

if(PANDOC)
	message(STATUS "Pandoc found - generating documentation for the CLI tools")
	#configuring manpage install dir
	if(WIN32 AND NOT CYGWIN)
		#put the manuals in the same directory as the binaries on windows
		set(CLIDOC_INSTALL_DIR ${INSTALL_BIN_DIR})
		set(CLIDOC_OUTPUT_DIR "${CMAKE_BINARY_DIR}/doc-cli" )
	else()
		#use cmake to find the man pages directory on *NIX
		include(GNUInstallDirs)
		if(CMAKE_INSTALL_MANDIR)
			set(CLIDOC_INSTALL_DIR ${CMAKE_INSTALL_MANDIR})
		else()
            set(CLIDOC_INSTALL_DIR ${INSTALL_BIN_DIR})
			message(STATUS "Couldn't find the man install directory - man pages will not be installed properly")
		endif()
		set(CLIDOC_OUTPUT_DIR "${CMAKE_BINARY_DIR}/man8" )
	endif()
	
else()
	message(STATUS "Pandoc not found - documentation will not be generated for the CLI tools")
endif()

#TODO will the man-pages be installed correctly on OS X?
foreach( e ${_cli_files} )
	add_executable( ${e} "${e}.cxx" )
	target_link_libraries( ${e} statismo_core ${ITK_LIBRARIES} ${Boost_LIBRARIES})

	install(TARGETS ${e} 
		DESTINATION ${INSTALL_BIN_DIR}
	)
  
	if(PANDOC AND CLIDOC_INSTALL_DIR)
		message(STATUS "Creating documentation for ${e}")
		
		set(CLIDOC_MD_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/${e}.md)
		if(WIN32 AND NOT CYGWIN)
			set(CLIDOC_OUT_FILE ${CLIDOC_OUTPUT_DIR}/${e}.html)
		else()
			set(CLIDOC_OUT_FILE ${CLIDOC_OUTPUT_DIR}/${e}.8)
		endif()
		
		add_custom_command(
			OUTPUT ${CLIDOC_OUT_FILE}
			COMMAND ${CMAKE_COMMAND} -E make_directory ${CLIDOC_OUTPUT_DIR}
			# a '/' instead of a '\' in the inputfile path on windows leads to pandoc failing to automatically recoginze the file format -> we have specify it manually
			COMMAND ${PANDOC} -f markdown -s ${CLIDOC_MD_SOURCE} -o ${CLIDOC_OUT_FILE}
			MAIN_DEPENDENCY ${CLIDOC_MD_SOURCE}
		)
		
		#build the cli documentation by default
		add_custom_target(doc-cli ALL
			DEPENDS ${CLIDOC_OUT_FILE}
			SOURCES ${CLIDOC_MD_SOURCE}
		)

		#install the entire cli documentation directory
		install(DIRECTORY ${CLIDOC_OUTPUT_DIR}
			DESTINATION ${CLIDOC_INSTALL_DIR}
		)


	endif()
endforeach()

