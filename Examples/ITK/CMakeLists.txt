CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(ssm_itk)
SET(CMAKE_VERBOSE_MAKEFILE ON)


FIND_PACKAGE(ITK 4.2 REQUIRED)
include (${ITK_USE_FILE})

    
SET (ITK_STATISTICAL_MODEL_INCLUDE_DIR ${STATISMO_ROOT_DIR}/ITKstatismo)

FILE(GLOB INCLUDE_FILES *.h)
FILE(GLOB SOURCE_FILES  *.txx )

include_directories(${STATISMO_ROOT_DIR} ${STATISMO_ROOT_DIR}/statismo_ITK  ${STATISMO_ROOT_DIR}/3rdParty  ${STATISMO_INCLUDE_DIR}  
					${ITK_INCLUDE_DIRS}  ${STATISMO_ROOT_DIR}/Representers/ITK
					${HDF5_INCLUDE_DIR} ${HDF5_INCLUDE_DIR_CPP}	
					)
link_directories(${ITK_LIBRARY_DIRS} ${STATISMO_LIBRARY_DIR}  ${HDF5_LIBRARY_DIR})

add_executable (buildDeformationModel buildDeformationModel.cpp 
	${SOURCE_FILES}
	${INCLUDE_FILES}
) 
ADD_DEPENDENCIES(buildDeformationModel HDF5)
target_link_libraries (buildDeformationModel ${ITK_LIBRARIES} ${STATISMO_LIBRARIES})

add_executable (deformationModelFitting deformationModelFitting.cpp 
	${SOURCE_FILES}
	${INCLUDE_FILES}
) 
ADD_DEPENDENCIES(deformationModelFitting HDF5)
target_link_libraries (deformationModelFitting  ${STATISMO_LIBRARIES} ${ITK_LIBRARIES})

add_executable (buildShapeModel buildShapeModel.cpp 
	${SOURCE_FILES}
	${INCLUDE_FILES}
) 
ADD_DEPENDENCIES(buildShapeModel HDF5)
target_link_libraries (buildShapeModel ${ITK_LIBRARIES} ${STATISMO_LIBRARIES})


add_executable (shapeModelFitting shapeModelFitting.cpp 
	${SOURCE_FILES}
	${INCLUDE_FILES}
) 
ADD_DEPENDENCIES(shapeModelFitting HDF5)
target_link_libraries (shapeModelFitting  ${STATISMO_LIBRARIES} ${ITK_LIBRARIES})
