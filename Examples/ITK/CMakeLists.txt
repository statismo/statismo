CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
PROJECT(ssm_itk)
SET(CMAKE_VERBOSE_MAKEFILE ON)


FIND_PACKAGE(ITK 4.1 REQUIRED)
include (${ITK_USE_FILE})

IF(MSVC11) #i.e. Visual Studio 2012 
  # Fix for VS2012 that has _VARIADIC_MAX set to 5. Don't set too high because it increases compiler memory usage / compile-time.
  add_definitions( -D_VARIADIC_MAX=10 )
  # Fix for another VS2012 problem: not all TR1 options are automatically detected, therefore we force them here.
  ADD_DEFINITIONS( -D BOOST_HAS_TR1 )
  ADD_DEFINITIONS( -D BOOST_NO_0X_HDR_INITIALIZER_LIST )
ENDIF()

    
SET (ITK_STATISTICAL_MODEL_INCLUDE_DIR ${STATISMO_ROOT_DIR}/ITKstatismo)

FILE(GLOB INCLUDE_FILES *.h)
FILE(GLOB SOURCE_FILES  *.txx )

include_directories(${STATISMO_ROOT_DIR} ${STATISMO_ROOT_DIR}/statismo_ITK  ${STATISMO_ROOT_DIR}/3rdParty  ${STATISMO_INCLUDE_DIR}  
                                        ${ITK_INCLUDE_DIRS} ${ITK_INCLUDE_DIRS}/itkhdf5 ${ITK_INCLUDE_DIRS}/itkhdf5/cpp ${STATISMO_ROOT_DIR}/Representers/ITK)


link_directories( ${ITK_LIBRARY_DIRS} ${STATISMO_LIBRARY_DIR} )

add_executable (itkBuildDeformationModel buildDeformationModel.cpp 
	${SOURCE_FILES}
	${INCLUDE_FILES}
) 

target_link_libraries (itkBuildDeformationModel ${ITK_LIBRARIES})

add_executable (itkDeformationModelFitting deformationModelFitting.cpp 
	${SOURCE_FILES}
	${INCLUDE_FILES}
) 

target_link_libraries (itkDeformationModelFitting   ${ITK_LIBRARIES})

add_executable (itkBuildShapeModel buildShapeModel.cpp 
	${SOURCE_FILES}
	${INCLUDE_FILES}
) 

target_link_libraries (itkBuildShapeModel ${ITK_LIBRARIES} )

add_executable (itkBuildShapeModel_75pcvar buildShapeModel_75pcvar.cpp 
	${SOURCE_FILES}
	${INCLUDE_FILES}
) 

target_link_libraries (itkBuildShapeModel_75pcvar ${ITK_LIBRARIES} )

add_executable (itkBuildGaussianProcessDeformationModel buildGaussianProcessDeformationModel.cpp 
	${SOURCE_FILES}
	${INCLUDE_FILES}
) 

target_link_libraries (itkBuildGaussianProcessDeformationModel ${ITK_LIBRARIES} )




add_executable (itkShapeModelFitting shapeModelFitting.cpp 
	${SOURCE_FILES}
	${INCLUDE_FILES}
) 

target_link_libraries (itkShapeModelFitting  ${ITK_LIBRARIES})

add_executable (itkLandmarkConstrainedShapeModelFitting landmarkConstrainedShapeModelFitting.cpp 
	${SOURCE_FILES}
	${INCLUDE_FILES}
) 

target_link_libraries (itkLandmarkConstrainedShapeModelFitting  ${ITK_LIBRARIES})




add_executable(itkLowRankGaussianProcessImageToImageRegistration lowRankGaussianProcessImageToImageRegistration.cpp
        ${SOURCE_FILES}
        ${INCLUDE_FILES}
) 

target_link_libraries (itkLowRankGaussianProcessImageToImageRegistration  ${ITK_LIBRARIES})

add_executable(itkSimpleGaussianProcessImageToImageRegistration simpleGaussianProcessImageToImageRegistration.cpp
        ${SOURCE_FILES}
        ${INCLUDE_FILES}
) 

target_link_libraries (itkSimpleGaussianProcessImageToImageRegistration  ${ITK_LIBRARIES})



INSTALL(TARGETS 
itkBuildDeformationModel  
itkDeformationModelFitting 
itkBuildShapeModel 
itkBuildShapeModel_75pcvar 
itkShapeModelFitting 
itkLandmarkConstrainedShapeModelFitting 
itkBuildGaussianProcessDeformationModel
itkLowRankGaussianProcessImageToImageRegistration
itkSimpleGaussianProcessImageToImageRegistration
RUNTIME DESTINATION bin)
INSTALL(PROGRAMS runITKExamples.sh DESTINATION  bin)
