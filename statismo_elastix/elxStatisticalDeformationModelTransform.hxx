/*======================================================================

  This file is part of the elastix software.

  Copyright (c) University Medical Center Utrecht. All rights reserved.
  See src/CopyrightElastix.txt or http://elastix.isi.uu.nl/legal.php for
  details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE. See the above copyright notices for more information.

======================================================================*/

#ifndef __elxStatisticalDeformationModelTransform_HXX_
#define __elxStatisticalDeformationModelTransform_HXX_

#include "elxStatisticalDeformationModelTransform.h"
#include <itkTransformixInputPointFileReader.h>
#include <itkOptimizerParameters.h>

namespace elastix
{

  /**
   * ********************* Constructor ****************************
   */

  template <class TElastix>
    StatisticalDeformationModelTransformElastix<TElastix>
    ::StatisticalDeformationModelTransformElastix()
  {
    this->m_StatisticalDeformationModelTransform =
      StatisticalDeformationModelTransformType::New();
    this->SetCurrentTransform( this->m_StatisticalDeformationModelTransform );
  } // end Constructor


  /*
   * ******************* BeforeRegistration ***********************
   */

  template <class TElastix>
    void StatisticalDeformationModelTransformElastix<TElastix>
    ::BeforeRegistration(void)
  {
  	/** Give initial parameters to this->m_Registration.*/
    this->InitializeTransform();

  } // end BeforeRegistration

  template <class TElastix>
    int StatisticalDeformationModelTransformElastix<TElastix>
    ::BeforeAllTransformix(void)
  {

    BeforeRegistration();
    return 0;

  } // end BeforeAllTransformix


  /**
   * ************************* InitializeTransform *********************
   */

  template <class TElastix>
    void StatisticalDeformationModelTransformElastix<TElastix>
    ::InitializeTransform( void )
  {

  	std::cout << "InitializeTransform" << std::endl;

  	// Read original statistical model. If no initialization with
  	// point values is performed, this one is used in the registration.
    this->GetConfiguration()->ReadParameter( m_StatisticalModelName,
      "StatisticalModelName", 0);
    typename StatisticalModelType::Pointer statisticalModel = StatisticalModelType::New();
    statisticalModel->Load(m_StatisticalModelName.c_str());


    /** Check if user wants automatic transform initialization; false by default. */
    bool automaticTransformInitialization = false;
    bool useLandmarksInRegistration = false;
    this->m_Configuration->ReadParameter( useLandmarksInRegistration, "UseLandmarksInRegistration", 0 );
    bool onlyInitializeWithLandmarks = false;
    this->m_Configuration->ReadParameter(onlyInitializeWithLandmarks, "OnlyInitializeWithLandmarks", 0);


    if ( (useLandmarksInRegistration || onlyInitializeWithLandmarks) && this->Superclass1::GetInitialTransform() == 0 )
    {
      automaticTransformInitialization = true;
    }

    /**
     * Run the itkTransformInitializer if:
     *  the user asked for AutomaticTransformInitialization
     */
    if ( automaticTransformInitialization )
    {
    	std::string fixedLandmarksFileName;
    	std::string movingLandmarksFileName;

    	this->m_Configuration->ReadParameter(fixedLandmarksFileName,"FixedLandmarks",0);
    	this->m_Configuration->ReadParameter(movingLandmarksFileName,"MovingLandmarks",0);

    	double variance;
    	this->m_Configuration->ReadParameter(variance, "PartiallyFixedModelVariance",0);

    	std::vector<InputPointType> fixedLandmarks = this->ReadPointSetFromFile(fixedLandmarksFileName);
    	std::vector<InputPointType> movingLandmarks = this->ReadPointSetFromFile(movingLandmarksFileName);

    	// The model used in the transform is the partially fixed one.
    	m_StatisticalModel = ComputePartiallyFixedModel(statisticalModel, fixedLandmarks, movingLandmarks, variance);
    }
    else {
    	// Without landmarks, we use the original model.
    	this->m_StatisticalModel = statisticalModel;
    }

    if(onlyInitializeWithLandmarks) {
    	// Get the coefficients of the partially fixed model's mean in terms of the original model
    	typename StatisticalModelType::VectorType coefficientsOfPartialModelMean
    		= statisticalModel->ComputeCoefficientsForDataset(m_StatisticalModel->DrawMean());
    	// Now forget about the partially fixed model:
    	this->m_StatisticalModel = statisticalModel;
    	this->m_StatisticalDeformationModelTransform->SetStatisticalModel(m_StatisticalModel);
    	this->m_StatisticalDeformationModelTransform->SetIdentity();


    	itk::OptimizerParameters<double> initialParameters;


    	//initialParameters = coefficientsOfPartialModelMean;

    	//this->m_StatisticalDeformationModelTransform->SetParameters(initialParameters);

    }
    else {
    	// Initialize the transform
    	this->m_StatisticalDeformationModelTransform->SetStatisticalModel(m_StatisticalModel);
    	this->m_StatisticalDeformationModelTransform->SetIdentity();
    }

    /** Set the initial parameters in this->m_Registration.*/
    if(this->m_Registration)
    	this->m_Registration->GetAsITKBaseType()->
      SetInitialTransformParameters( this->GetParameters() );


  } // end InitializeTransform


  template <class TElastix>
  typename StatisticalDeformationModelTransformElastix<TElastix>::StatisticalModelType::Pointer
  StatisticalDeformationModelTransformElastix<TElastix>
  ::ComputePartiallyFixedModel(
  const StatisticalModelType* statisticalModel,
  const std::vector<InputPointType>& fixedLandmarks,
  const std::vector<InputPointType>& movingLandmarks,
  double variance
  ) const
  {
     typename StatisticalModelType::PointValueListType constraints;
     for (unsigned i = 0; i < movingLandmarks.size(); i++) {

			// compute the diff
			typename RepresenterType::ValueType displacement;
			for (unsigned d = 0; d < RepresenterType::Dimensions; d++) {
				displacement[d] = (movingLandmarks[i][d] - fixedLandmarks[i][d]);
			}
			typename RepresenterType::PointType pt = fixedLandmarks[i];
			constraints.push_back(typename StatisticalModelType::PointValuePairType(pt, displacement));
     }


     typename PartiallyFixedModelBuilderType::Pointer partiallyFixedModelBuilder = PartiallyFixedModelBuilderType::New();


     typename StatisticalModelType::Pointer partiallyFixedModel
			 = partiallyFixedModelBuilder->BuildNewModelFromModel(statisticalModel, constraints, variance, false);


     return partiallyFixedModel;
	}


  /**
   * ************** TransformPointsSomePoints *********************
   *
	 * Reads a set of input points
	 *
   */

  template <class TElastix>
  std::vector<typename StatisticalDeformationModelTransformElastix<TElastix>::InputPointType>
  StatisticalDeformationModelTransformElastix<TElastix>
  ::ReadPointSetFromFile( const std::string filename ) const
  {
    /** Typedef's. */
    typedef bool                                          DummyIPPPixelType;
    typedef itk::DefaultStaticMeshTraits<
      DummyIPPPixelType, Superclass2::FixedImageDimension,
      Superclass2::FixedImageDimension, CoordRepType>                  MeshTraitsType;
    typedef itk::PointSet< DummyIPPPixelType,
    		Superclass2::FixedImageDimension, MeshTraitsType>                PointSetType;
    typedef itk::TransformixInputPointFileReader<
      PointSetType >                                      IPPReaderType;
    typedef itk::Vector< float, Superclass2::FixedImageDimension >     DeformationVectorType;

    /** Construct an ipp-file reader. */
    typename IPPReaderType::Pointer ippReader = IPPReaderType::New();
    ippReader->SetFileName( filename.c_str() );

    /** Read the input points. */
    elxout << "  Reading input point file: " << filename << std::endl;
    try
    {
      ippReader->Update();
    }
    catch (itk::ExceptionObject & err)
    {
      xl::xout["error"] << "  Error while opening input point file." << std::endl;
      xl::xout["error"] << err << std::endl;
    }

    /** Some user-feedback. */
    if ( ippReader->GetPointsAreIndices() )
    {
      elxout << "  Input points are specified as image indices." << std::endl;
      itkExceptionMacro("For now, points HAVE to be specified in coordinates.");
    }
    else
    {
      elxout << "  Input points are specified in world coordinates." << std::endl;
    }
    unsigned int nrofpoints = ippReader->GetNumberOfPoints();
    elxout << "  Number of specified input points: " << nrofpoints << std::endl;

    /** Get the set of input points. */
    typename PointSetType::Pointer inputPointSet = ippReader->GetOutput();

    return inputPointSet->GetPoints()->CastToSTLContainer();

  }

  /**
   * ************************* WriteToFile ************************
   *
   * Saves the TransformParameters as a vector and if wanted
   * also as a deformation field.
   */

  template <class TElastix>
  void StatisticalDeformationModelTransformElastix<TElastix>
  ::WriteToFile( const ParametersType & param ) const
  {
    /** Call the WriteToFile from the TransformBase. */
    this->Superclass2::WriteToFile( param );

    /** Add some BSplineTransform specific lines. */
    xout["transpar"] << std::endl << "// StatsisticalDeformationModel specific" << std::endl;

    xout["transpar"] << "(StatisticalModelName \"" << m_StatisticalModelName << "\")" << std::endl;

  } // end WriteToFile()



} // end namespace elastix


#endif // end #ifndef __elxStatisticalDeformationModelTransform_HXX_

