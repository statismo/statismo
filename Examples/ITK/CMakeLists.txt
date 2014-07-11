cmake_minimum_required(VERSION 2.6)

project(ssm_itk)

find_package(ITK 4.1 REQUIRED)
include (${ITK_USE_FILE})

if(MSVC11) #i.e. Visual Studio 2012
  # Fix for VS2012 that has _VARIADIC_MAX set to 5. Don't set too high because it increases compiler memory usage / compile-time.
  add_definitions( -D_VARIADIC_MAX=10 )
  # Fix for another VS2012 problem: not all TR1 options are automatically detected, therefore we force them here.
  add_definitions( -D BOOST_HAS_TR1 )
  add_definitions( -D BOOST_NO_0X_HDR_INITIALIZER_LIST )
endif()

set(ITK_STATISTICAL_MODEL_INCLUDE_DIR ${STATISMO_ROOT_DIR}/ITKstatismo)

file(GLOB INCLUDE_FILES *.h)
file(GLOB SOURCE_FILES  *.txx )

# ITK comes with its own version of HDF5. As we want to use the version that comes with ITK,
# we have to find the directories where the include files are located and set the inlcude dir accordingly
find_path(ITK_HDF5_INCLUDE_DIR hdf5.h PATHS ${ITK_INCLUDE_DIRS} PATH_SUFFIXES itkhdf5 itkhdf5/cpp )
find_path(ITK_HDF5_INCLUDE_DIR_CPP H5Cpp.h PATHS ${ITK_INCLUDE_DIRS} PATH_SUFFIXES itkhdf5 itkhdf5/cpp )

include_directories(
  ${STATISMO_ROOT_DIR}
  ${STATISMO_ROOT_DIR}/statismo_ITK
  ${STATISMO_INCLUDE_DIR}
  ${ITK_INCLUDE_DIRS}
  ${ITK_HDF5_INCLUDE_DIR}
  ${ITK_HDF5_INCLUDE_DIR_CPP}
  ${STATISMO_ROOT_DIR}/Representers/ITK
)


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

install(TARGETS
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
install(PROGRAMS runITKExamples.sh DESTINATION  bin)
