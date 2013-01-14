/*======================================================================

  This file is part of the elastix software.

  Copyright (c) University Medical Center Utrecht. All rights reserved.
  See src/CopyrightElastix.txt or http://elastix.isi.uu.nl/legal.php for
  details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE. See the above copyright notices for more information.

======================================================================*/

#ifndef __elxStatisticalDeformationModelTransform_H_
#define __elxStatisticalDeformationModelTransform_H_
#include "itkAdvancedStatisticalDeformationModelTransform.h"


#include "itkStatisticalModel.h"
#include "statismo_ITK/itkPartiallyFixedModelBuilder.h"
#include "Representers/ITK/itkVectorImageLMAlignRepresenter.h"
#include "elxIncludes.h"
#include "itkImage.h"
#include "itkVector.h"
#include <vector>

namespace elastix
{

  /**
   * \class StatisticalDeformationModelTransformElastix
   * \brief A transform based on the itk::StatisticalDeformationModelTransform.
   *
   * This transform is a translation transformation.
   *
   * The parameters used in this class are:
   * \parameter Transform: Select this transform as follows:\n
   *    <tt>(%Transform "StatisticalDeformationModelTransform")</tt>
   * \parameter AutomaticTransformInitialization: whether or not the initial translation
   *    between images should be estimated as the distance between their centers.\n
   *    example: <tt>(AutomaticTransformInitialization "true")</tt> \n
   *    By default "false" is assumed. So, no initial translation.
   * \parameter AutomaticTransformInitializationMethod: how to initialize this
   *    transform. Should be one of {GeometricalCenter, CenterOfGravity}.\n
   *    example: <tt>(AutomaticTransformInitializationMethod "CenterOfGravity")</tt> \n
   *    By default "GeometricalCenter" is assumed.\n
   *
   * \ingroup Transforms
   */

  template < class TElastix >
    class StatisticalDeformationModelTransformElastix:
      public itk::AdvancedCombinationTransform<
          typename elx::TransformBase<TElastix>::CoordRepType,
          elx::TransformBase<TElastix>::FixedImageDimension > ,
      public elx::TransformBase<TElastix>
  {
  public:

    /** Standard ITK-stuff. */
    typedef StatisticalDeformationModelTransformElastix                     Self;

    typedef itk::AdvancedCombinationTransform<
      typename elx::TransformBase<TElastix>::CoordRepType,
      elx::TransformBase<TElastix>::FixedImageDimension >   Superclass1;

    typedef elx::TransformBase<TElastix>                    Superclass2;

    typedef itk::SmartPointer<Self>                         Pointer;
    typedef itk::SmartPointer<const Self>                   ConstPointer;

    /** Method for creation through the object factory. */
    itkNewMacro( Self );

    /** Run-time type information (and related methods). */
    itkTypeMacro( StatisticalDeformationModelTransformElastix, itk::AdvancedCombinationTransform );

    /** Name of this class.
     * Use this name in the parameter file to select this specific transform. \n
     * example: <tt>(Transform "StatisticalDeformationModelTransform")</tt>\n
     */
    elxClassNameMacro( "StatisticalDeformationModelTransform" );

    /** Dimension of the domain space. */
    itkStaticConstMacro( SpaceDimension, unsigned int, Superclass2::FixedImageDimension );

    /** Typedefs inherited from the superclass. */
    typedef typename Superclass1::ScalarType                ScalarType;
    typedef typename Superclass1::ParametersType            ParametersType;
    typedef typename Superclass1::JacobianType              JacobianType;
    typedef typename Superclass1::InputVectorType           InputVectorType;
    typedef typename Superclass1::OutputVectorType          OutputVectorType;
    typedef typename Superclass1::InputCovariantVectorType  InputCovariantVectorType;
    typedef typename Superclass1::OutputCovariantVectorType OutputCovariantVectorType;
    typedef typename Superclass1::InputVnlVectorType        InputVnlVectorType;
    typedef typename Superclass1::OutputVnlVectorType       OutputVnlVectorType;
    typedef typename Superclass1::InputPointType            InputPointType;
    typedef typename Superclass1::OutputPointType           OutputPointType;

    /** Typedef's from the TransformBase class. */
    typedef typename Superclass2::ElastixType               ElastixType;
    typedef typename Superclass2::ElastixPointer            ElastixPointer;
    typedef typename Superclass2::ConfigurationType         ConfigurationType;
    typedef typename Superclass2::ConfigurationPointer      ConfigurationPointer;
    typedef typename Superclass2::RegistrationType          RegistrationType;
    typedef typename Superclass2::RegistrationPointer       RegistrationPointer;
    typedef typename Superclass2::CoordRepType              CoordRepType;
    typedef typename Superclass2::FixedImageType            FixedImageType;
    typedef typename Superclass2::MovingImageType           MovingImageType;
    typedef typename Superclass2::ITKBaseType               ITKBaseType;
    typedef typename Superclass2::CombinationTransformType  CombinationTransformType;

    //typedef typename Superclass2::FixedImageDimension FixedImageDimension;
    /** Statismo typedefs */
    typedef itk::VectorImageLMAlignRepresenter<CoordRepType, elx::TransformBase<TElastix>::FixedImageDimension, elx::TransformBase<TElastix>::FixedImageDimension> RepresenterType;
    typedef itk::StatisticalModel<RepresenterType> StatisticalModelType;

    /** The ITK-class that provides most of the functionality, and
     * that is set as the "CurrentTransform" in the CombinationTransform */
    typedef itk::AdvancedStatisticalDeformationModelTransform<
      RepresenterType, CoordRepType, elx::TransformBase<TElastix>::FixedImageDimension >   StatisticalDeformationModelTransformType;
    typedef typename StatisticalDeformationModelTransformType::Pointer      StatisticalDeformationModelTransformPointer;
    typedef typename itk::PartiallyFixedModelBuilder<RepresenterType> 			PartiallyFixedModelBuilderType;


//    /** Extra typedefs */
//    typedef itk::StatisticalDeformationModelTransformInitializer<
//      StatisticalDeformationModelTransformType,
//      FixedImageType,
//      MovingImageType>                                      TransformInitializerType;
//    typedef typename TransformInitializerType::Pointer      TransformInitializerPointer;

    /** Execute stuff before the actual registration:
     * \li Call InitializeTransform.
     */
    virtual void BeforeRegistration(void);

    virtual int BeforeAllTransformix(void);

    /** Initialize Transform.
     * \li Set all parameters to zero.
     * \li Set initial translation:
     *  the initial translation between fixed and moving image is guessed,
     *  if the user has set (AutomaticTransformInitialization "true").
     */
    virtual void InitializeTransform(void);


    /** Function to read transform-parameters from a file. */
    //virtual void ReadFromFile( void );

    /** Function to write transform-parameters to a file. */
    virtual void WriteToFile( const ParametersType & param ) const;

    /** Does what it says. */
    std::vector<InputPointType> ReadPointSetFromFile( const std::string filename ) const;

    typename StatisticalModelType::Pointer ComputePartiallyFixedModel(
    const StatisticalModelType* statisticalModel,
    const std::vector<InputPointType>& fixedLandmarks,
    const std::vector<InputPointType>& movingLandmarks,
    double variance
    ) const;


  protected:

    /** The constructor. */
    StatisticalDeformationModelTransformElastix();
    /** The destructor. */
    virtual ~StatisticalDeformationModelTransformElastix() {};

    StatisticalDeformationModelTransformPointer m_StatisticalDeformationModelTransform;
    typename StatisticalModelType::Pointer m_StatisticalModel;
    std::string m_StatisticalModelName;

  private:

    /** The private constructor. */
    StatisticalDeformationModelTransformElastix( const Self& ); // purposely not implemented
    /** The private copy constructor. */
    void operator=( const Self& );              // purposely not implemented

  }; // end class StatisticalDeformationModelTransformElastix


} // end namespace elastix

#ifndef ITK_MANUAL_INSTANTIATION
#include "elxStatisticalDeformationModelTransform.hxx"
#endif

#endif // end #ifndef __elxStatisticalDeformationModelTransform_H_
