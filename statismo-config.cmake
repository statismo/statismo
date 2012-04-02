
# hdf5 stuff

SET (HDF5_INCLUDE_DIR ${STATISMO_DIR}/3rdParty/hdf5/include)
SET (HDF5_INCLUDE_DIR_CPP ${HDF5_INCLUDE_DIR}/cpp)

IF(${CMAKE_SYSTEM_NAME} MATCHES "WIN32")
	if(CMAKE_SIZEOF_VOID_P MATCHES 4)
		SET (HDF5_LIBRARY_DIR ${STATISMO_DIR}/3rdParty/hdf5/lib/win32)			
   else()
		SET (HDF5_LIBRARY_DIR ${STATISMO_DIR}/3rdParty/hdf5/lib/win64)
   endif()
	SET (HDF5_LIBRARIES hdf5 hdf5_cpp)
ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	if(CMAKE_SIZEOF_VOID_P MATCHES 4)	
		SET (HDF5_LIBRARY_DIR ${STATISMO_DIR}/3rdParty/hdf5/lib/linux32)
	else()
		SET (HDF5_LIBRARY_DIR ${STATISMO_DIR}/3rdParty/hdf5/lib/linux64)		
	endif()
	SET (HDF5_LIBRARIES hdf5 hdf5_cpp)
ELSE()
	message(FATAL_ERROR 
		"The system ${CMAKE_SYSTEM_NAME} is currently not supported by the build system."
		"The problem is that the hdf5 libraries are not yet included in the statismo distribution."
		"You can change this yourself, by copy the necessary hdf5 libraries in ${STATISMO_DIR}/3rdParty/hdf5/lib/YOUR_SYSTEM"
		" and adapt the ${STATISMO_DIR}/statismo-config.cmake file. Please also report to the developers,"
		"such that your system can be supported in future versions")
ENDIF() 

SET(STATISMO_LIBRARIES   ${HDF5_LIBRARIES})
SET(STATISMO_INCLUDE_DIR ${STATISMO_DIR} ${STATISMO_DIR}/3rdParty ${HDF5_INCLUDE_DIR} ${HDF5_INCLUDE_DIR_CPP})
SET(STATISMO_LIBRARY_DIR  ${HDF5_LIBRARY_DIR})
	