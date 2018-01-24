if(MSVC11) #i.e. Visual Studio 2012
  # Fix for VS2012 that has _VARIADIC_MAX set to 5. Don't set too high because it increases compiler memory usage / compile-time.
  add_definitions( -D_VARIADIC_MAX=10 )
endif()

set( _examples
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

foreach( e ${_examples} )
  add_executable( ${e} ${e}.cxx
  )
  target_link_libraries( ${e} statismo_core ${ITK_LIBRARIES} )
endforeach()

