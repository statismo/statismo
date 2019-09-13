set(TARGET_EXAMPLES
  itkBuildGaussianProcessDeformationModel
  itkBuildShapeModel_75pcvar
  itkLandmarkConstrainedShapeModelFitting
  itkShapeModelFitting
  itkBuildDeformationModel
  itkBuildShapeModel
  itkDeformationModelFitting
  itkLowRankGaussianProcessImageToImageRegistration
  itkSimpleGaussianProcessImageToImageRegistration
)

foreach(e ${TARGET_EXAMPLES})
  add_executable(${e} ${e}.cxx)
  target_link_libraries(${e} statismo_ITK statismo_core)
endforeach()