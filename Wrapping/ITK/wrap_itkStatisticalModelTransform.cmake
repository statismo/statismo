wrap_include("itkVectorImageRepresenter.h")

WRAP_CLASS("itk::StatisticalModelTransform" POINTER)
	WRAP_TEMPLATE("VIRF22D2" "itk::VectorImageRepresenter<float, 2, 2 >, double, 2")
	WRAP_TEMPLATE("VIRF33D3" "itk::VectorImageRepresenter<float, 3, 3 >, double, 3")
END_WRAP_CLASS()
