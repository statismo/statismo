wrap_include("itkImageRepresenter.h")
wrap_include("itkVectorImageRepresenter.h")


WRAP_CLASS("itk::PCAModelBuilder" POINTER)
	WRAP_TEMPLATE("IRF2" "itk::ImageRepresenter<float, 2 >")
	WRAP_TEMPLATE("VIRF22" "itk::VectorImageRepresenter<float, 2, 2 >")
	WRAP_TEMPLATE("VIRF33" "itk::VectorImageRepresenter<float, 3, 3 >")
END_WRAP_CLASS()
