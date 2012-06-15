
FIND_PACKAGE(WrapITK REQUIRED)

SET(PROJECT_SOURCE_DIR ${STATISMO_ROOT_DIR} ${STATISMO_ROOT_DIR}/statismo_ITK)

SET(WRAPPER_ITK_MODLE_DIR ${ITK_DIR}/Wrapping/Typedefs)

include_directories(${WRAPPER_ITK_MODLE_DIR}
					${STATISMO_ROOT_DIR} ${STATISMO_ROOT_DIR}/statismo_ITK  ${STATISMO_ROOT_DIR}/3rdParty  ${STATISMO_INCLUDE_DIRS}  
					${ITK_INCLUDE_DIRS}  ${STATISMO_ROOT_DIR}/Representers/ITK
					${HDF5_INCLUDE_DIR} ${HDF5_INCLUDE_DIR_CPP})
link_directories(${ITK_LIBRARY_DIRS} ${STATISMO_LIBRARY_DIR} ${HDF5_LIBRARY_DIR} )

itk_wrap_module("statismo_ITK") # name of python package
SET(WRAPPER_LIBRARY_DEPENDS ITKCommon ITKTransform ITKPyBase ITKIOImageBase ITKSpatialObjects ITKReview ITKMesh)

SET(WRAPPER_LIBRARY_LINK_LIBRARIES ${ITK_LIBRARIES})
itk_auto_load_submodules() # includes all *.wrap custom files
itk_end_wrap_module()

