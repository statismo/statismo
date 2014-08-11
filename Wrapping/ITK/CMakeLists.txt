find_package( WrapITK REQUIRED )

set( PROJECT_SOURCE_DIR
  ${STATISMO_ROOT_DIR}
  ${STATISMO_ROOT_DIR}/statismo_ITK
)
set( WRAPPER_ITK_MODULE_DIR
  ${ITK_DIR}/WrapITK/Configuration/Typedefs
)

include_directories(
  ${WRAPPER_ITK_MODULE_DIR}
  ${STATISMO_ROOT_DIR}
  ${STATISMO_ROOT_DIR}/statismo_ITK
  ${STATISMO_INCLUDE_DIRS}
  ${ITK_INCLUDE_DIRS}
  ${STATISMO_ROOT_DIR}/Representers/ITK
  ${HDF5_INCLUDE_DIR}
  ${HDF5_INCLUDE_DIR_CPP}
)
link_directories(
  ${ITK_LIBRARY_DIRS}
  ${STATISMO_LIBRARY_DIR}
  ${HDF5_LIBRARY_DIR}
)

# name of python package
itk_wrap_module("statismo_ITK")
set( WRAPPER_LIBRARY_DEPENDS
  ITKCommon
  ITKTransform
  ITKPyBase
  ITKIOImageBase
  ITKSpatialObjects
  ITKReview
  ITKMesh
)

set( WRAPPER_LIBRARY_LINK_LIBRARIES ${ITK_LIBRARIES} )
itk_auto_load_submodules() # includes all *.wrap custom files
itk_end_wrap_module()

