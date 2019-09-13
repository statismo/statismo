set(TARGET_EXAMPLES
  vtkBasicSamplingExample
  vtkBuildConditionalModelExample
  vtkBuildGaussianProcessShapeModelExample
  vtkBuildIntensityModelExample
  vtkBuildPosteriorModelExample
  vtkBuildShapeModelExample
  vtkCrossValidationExample
  vtkReduceModelVarianceExample
  vtkSpatiallyVaryingGPModelExample
)

foreach(e ${TARGET_EXAMPLES})
  add_executable(${e} ${e}.cxx)
  target_link_libraries(${e} statismo_VTK statismo_core)
endforeach()

