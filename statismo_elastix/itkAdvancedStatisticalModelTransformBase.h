/*======================================================================

This file is part of the elastix software.

Copyright (c) University Medical Center Utrecht. All rights reserved.
See src/CopyrightElastix.txt or http://elastix.isi.uu.nl/legal.php for
details.

This software is distributed WITHOUT ANY WARRANTY; without even
the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
PURPOSE. See the above copyright notices for more information.

======================================================================*/

/*

  Program:   Insight Segmentation & Registration Toolkit
  Module:    $RCSfile: itkAdvancedStatisticalModelTransformBase.h,v $
  Language:  C++
  Date:      $Date: 2008-06-29 12:58:58 $
  Version:   $Revision: 1.20 $

  Copyright (c) Insight Software Consortium. All rights reserved.
  See ITKCopyright.txt or http://www.itk.org/HTML/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/

#ifndef __itkAdvancedStatisticalModelTransformBase_h
#define __itkAdvancedStatisticalModelTransformBase_h

#include <iostream>

#include "itkStatisticalModel.h"
#include "itkMatrix.h"
#include "itkAdvancedTransform.h"
#include "itkExceptionObject.h"
#include "itkMacro.h"

namespace itk
{

/**
 * \brief Base class that implements an itk transform interface for statistical models.
 *
 * Statistical models (shape or deformation models) are often used to model the typical variations within
 * an object class. The StatisticalModelTransformBase implements the standard Transform interface, and thus allows
 * for the use of statistical models within the ITK registration framework.
 * Subclasses will need to implement the TransformPoint method, as its semantics depends on the type of statistical model.
 *
 * \ingroup Transforms
 *
 */

template <
	class TRepresenter,									// Statismo Representer
  class TScalarType=double,         // Data type for scalars
  unsigned int TDimension=3  // Number of dimensions in the input space
  >
class AdvancedStatisticalModelTransformBase
  : public AdvancedTransform< TScalarType, TDimension, TDimension >
{
public:
  /** Standard typedefs   */
  typedef AdvancedStatisticalModelTransformBase     Self;
  typedef AdvancedTransform< TScalarType,
    TDimension, TDimension >       Superclass;
  typedef SmartPointer<Self>                    Pointer;
  typedef SmartPointer<const Self>              ConstPointer;

  /** Run-time type information (and related methods). */
  itkTypeMacro( AdvancedStatisticalModelTransformBase, AdvancedTransform );


	typedef vnl_vector<statismo::ScalarType> VectorType;
  typedef vnl_matrix<statismo::ScalarType> MatrixType;

  /**
   * Copy the members of the current transform. To be used by subclasses in the CreateAnother method.
   */
  virtual void CopyBaseMembers(AdvancedStatisticalModelTransformBase* another) const
  {
 	  another->m_StatisticalModel = m_StatisticalModel;
 	  another->m_coeff_vector = m_coeff_vector;
 	  another->m_usedNumberCoefficients = m_usedNumberCoefficients;
 	  another->m_FixedParameters = m_FixedParameters;
 	  another->m_Parameters = this->m_Parameters;
   }


  /** Dimension of the domain space. */
  itkStaticConstMacro( InputSpaceDimension, unsigned int, TDimension );
  itkStaticConstMacro( OutputSpaceDimension, unsigned int, TDimension );
//  itkStaticConstMacro( ParametersDimension, unsigned int,
//    NOutputDimensions * ( NInputDimensions + 1 ) );

  /** Typedefs from the Superclass. */
  typedef typename Superclass::ScalarType           ScalarType;
  typedef typename Superclass::ParametersType       ParametersType;
  typedef typename Superclass::NumberOfParametersType NumberOfParametersType;
  typedef typename Superclass::JacobianType         JacobianType;
  typedef typename Superclass::InputVectorType      InputVectorType;
  typedef typename Superclass::OutputVectorType     OutputVectorType;
  typedef typename Superclass
    ::InputCovariantVectorType                      InputCovariantVectorType;
  typedef typename Superclass
    ::OutputCovariantVectorType                     OutputCovariantVectorType;
  typedef typename Superclass::InputVnlVectorType   InputVnlVectorType;
  typedef typename Superclass::OutputVnlVectorType  OutputVnlVectorType;
  typedef typename Superclass::InputPointType       InputPointType;
  typedef typename Superclass::OutputPointType      OutputPointType;

  typedef typename Superclass
    ::NonZeroJacobianIndicesType                    NonZeroJacobianIndicesType;
  typedef typename Superclass::SpatialJacobianType  SpatialJacobianType;
  typedef typename Superclass
    ::JacobianOfSpatialJacobianType                 JacobianOfSpatialJacobianType;
  typedef typename Superclass::SpatialHessianType   SpatialHessianType;
  typedef typename Superclass
    ::JacobianOfSpatialHessianType                  JacobianOfSpatialHessianType;
  typedef typename Superclass::InternalMatrixType   InternalMatrixType;

	typedef TRepresenter RepresenterType;
	typedef itk::StatisticalModel<RepresenterType> StatisticalModelType;

  /** Set the transformation to an Identity
   * This sets the matrix to identity and the Offset to null.
   */
  virtual void SetIdentity( void );

  /** Set the transformation from a container of parameters.
   * The first (NOutputDimension x NInputDimension) parameters define the
   * matrix and the last NOutputDimension parameters the translation.
   * Offset is updated based on current center.
   */
  void SetParameters( const ParametersType & parameters );

  /** Get the Transformation Parameters. */
  const ParametersType & GetParameters( void ) const;

  /** Set the fixed parameters and update internal transformation. */
  virtual void SetFixedParameters( const ParametersType & ){
		 // there no fixed parameters
	 }


  OutputPointType     TransformPoint( const InputPointType & point ) const = 0;
//  OutputVectorType    TransformVector( const InputVectorType & vector ) const;
//  OutputVnlVectorType TransformVector( const InputVnlVectorType & vector ) const;
//  OutputCovariantVectorType TransformCovariantVector(
//    const InputCovariantVectorType & vector ) const;


  /**
   * returns an empty Parameter vector, as the tranform does not have any fixed parameters
   */
  virtual const ParametersType& GetFixedParameters(void) const {
	  return this->m_FixedParameters;
  };

  /**
   * Convenience method to obtain the current coefficients of the StatisticalModel as a statismo::VectorType.
   * The resulting vector is the same as it would be obtained from GetParameters.
   */
  virtual VectorType GetCoefficients() const {
	  return m_coeff_vector;
  }

  /**
   * Convenicne method to set the coefficients of the underlying StatisticalModel from a statismo::VectorType.
   * This has the same effect as calling SetParameters.
   */
  virtual void SetCoefficients( VectorType& coefficients) { m_coeff_vector = coefficients; }

  /**
   * Set the statistical model that defines the valid transformations.
   */
void SetStatisticalModel(const StatisticalModelType* model);

/**
 * Returns the statistical model used.
 */
typename StatisticalModelType::ConstPointer GetStatisticalModel() const;

/**
 * Set the number of PCA Coefficients used by the model. This parameters has a
 * regularization effect. Setting it to a small value will restrict the possible tranformations
 * to the main modes of variations.
 */
void SetUsedNumberOfCoefficients(unsigned n) { m_usedNumberCoefficients = n; }

/**
 * returns the number of used model coefficients.
 */
unsigned GetUsedNumberOfCoefficients() { return m_usedNumberCoefficients; }


  /** Compute the Jacobian of the transformation. */
  virtual void GetJacobian(
    const InputPointType &,
    JacobianType &,
    NonZeroJacobianIndicesType & ) const;

  /** Compute the spatial Jacobian of the transformation. */
  virtual void GetSpatialJacobian(
    const InputPointType &,
    SpatialJacobianType & ) const
  {
    itkExceptionMacro( << "This function is not implemented." );
  };

  /** Compute the spatial Hessian of the transformation. */
  virtual void GetSpatialHessian(
    const InputPointType &,
    SpatialHessianType & ) const
  {
    itkExceptionMacro( << "This function is not implemented." );
  };

  /** Compute the Jacobian of the spatial Jacobian of the transformation. */
  virtual void GetJacobianOfSpatialJacobian(
    const InputPointType &,
    JacobianOfSpatialJacobianType &,
    NonZeroJacobianIndicesType & ) const
  {
    itkExceptionMacro( << "This function is not implemented." );
  };

  /** Compute the Jacobian of the spatial Jacobian of the transformation. */
  virtual void GetJacobianOfSpatialJacobian(
    const InputPointType &,
    SpatialJacobianType &,
    JacobianOfSpatialJacobianType &,
    NonZeroJacobianIndicesType & ) const
  {
    itkExceptionMacro( << "This function is not implemented." );
  };

  /** Compute the Jacobian of the spatial Hessian of the transformation. */
  virtual void GetJacobianOfSpatialHessian(
    const InputPointType &,
    JacobianOfSpatialHessianType &,
    NonZeroJacobianIndicesType & ) const
  {
    itkExceptionMacro( << "This function is not implemented." );
  };

  /** Compute both the spatial Hessian and the Jacobian of the
   * spatial Hessian of the transformation. */
  virtual void GetJacobianOfSpatialHessian(
    const InputPointType & ipp,
    SpatialHessianType & sh,
    JacobianOfSpatialHessianType & jsh,
    NonZeroJacobianIndicesType & nonZeroJacobianIndices ) const
  {
    itkExceptionMacro( << "This function is not implemented." );
  };

protected:


  //AdvancedStatisticalModelTransformBase( unsigned int paramDims );
  AdvancedStatisticalModelTransformBase();
	virtual ~AdvancedStatisticalModelTransformBase(){};

	void PrintSelf(std::ostream &os, Indent indent) const;

	typename StatisticalModelType::ConstPointer m_StatisticalModel;
	VectorType m_coeff_vector;
	unsigned m_usedNumberCoefficients;
	ParametersType m_FixedParameters;

private:

  AdvancedStatisticalModelTransformBase(const Self & other);
  const Self & operator=( const Self & );



}; //class AdvancedStatisticalModelTransformBase

}  // namespace itk

#if ITK_TEMPLATE_TXX
# include "itkAdvancedStatisticalModelTransformBase.txx"
#endif

#endif /* __itkAdvancedStatisticalModelTransformBase_h */

