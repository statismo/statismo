/*======================================================================

  This file is part of the elastix software.

  Copyright (c) University Medical Center Utrecht. All rights reserved.
  See src/CopyrightElastix.txt or http://elastix.isi.uu.nl/legal.php for
  details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE. See the above copyright notices for more information.

======================================================================*/

/*=========================================================================

  Program:   Insight Segmentation & Registration Toolkit
  Module:    $RCSfile: itkAdvancedStatisticalModelTransformBase.txx,v $
  Language:  C++
  Date:      $Date: 2008-06-29 12:58:58 $
  Version:   $Revision: 1.17 $

  Copyright (c) Insight Software Consortium. All rights reserved.
  See ITKCopyright.txt or http://www.itk.org/HTML/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/
#ifndef _itkAdvancedStatisticalModelTransformBase_txx
#define _itkAdvancedStatisticalModelTransformBase_txx

#include "itkNumericTraits.h"
#include "itkAdvancedStatisticalModelTransformBase.h"

namespace itk
{

// Constructor with default arguments
template < class TRepresenter, class TScalarType,  unsigned int TDimension >
AdvancedStatisticalModelTransformBase<TRepresenter, TScalarType, TDimension>
::AdvancedStatisticalModelTransformBase() :
	Superclass(0), // we don't know the number of parameters at this point.
	m_StatisticalModel(0),
	m_coeff_vector(0),
	m_usedNumberCoefficients(10000) // something large
{
	itkDebugMacro( << "Constructor MorphableModelTransform()");

	this->m_FixedParameters.SetSize(0);
}


/*!
 * Set the morphable model and ajust the parameters dimension.
 */
template < class TRepresenter, class TScalarType,  unsigned int TDimension >
void
AdvancedStatisticalModelTransformBase<TRepresenter,  TScalarType, TDimension>
::SetStatisticalModel(const StatisticalModelType* model)
{
	itkDebugMacro( << "Setting statistical model ");
	m_StatisticalModel = model;


	this->m_Parameters.SetSize(model->GetNumberOfPrincipalComponents());
	this->m_Parameters.Fill(0.0);

	this->m_coeff_vector.set_size(model->GetNumberOfPrincipalComponents());

}

template < class TRepresenter, class TScalarType,  unsigned int TDimension >
typename AdvancedStatisticalModelTransformBase<TRepresenter,  TScalarType, TDimension>::StatisticalModelType::ConstPointer
AdvancedStatisticalModelTransformBase<TRepresenter,  TScalarType, TDimension>
::GetStatisticalModel() const
{
	itkDebugMacro( << "Getting statistical model ");
	return m_StatisticalModel;
}



/*!
 * Set the parameters to the IdentityTransform.
 */
template < class TRepresenter, class TScalarType,  unsigned int TDimension >
void
AdvancedStatisticalModelTransformBase<TRepresenter,  TScalarType, TDimension>
::SetIdentity( )
{
	itkDebugMacro( << "Setting Identity");

	for (unsigned i = 0; i  < this->GetNumberOfParameters(); i++)
		this->m_coeff_vector[i] = 0;


	this->Modified();
}


template < class TRepresenter, class TScalarType,  unsigned int TDimension >
void
AdvancedStatisticalModelTransformBase<TRepresenter,  TScalarType, TDimension>
::SetParameters( const ParametersType & parameters )
{
  itkDebugMacro( << "Setting parameters " << parameters );

  // Set angle
  for(unsigned int i=0; i < std::min(m_usedNumberCoefficients, (unsigned) this->GetNumberOfParameters()); i++)
    {
	m_coeff_vector[i] = parameters[i];
    }
  for (unsigned int i = std::min(m_usedNumberCoefficients, (unsigned) this->GetNumberOfParameters()); i <  this->GetNumberOfParameters(); i++) {
	  m_coeff_vector[i] = 0;
  }

  // Modified is always called since we just have a pointer to the
  // parameters and cannot know if the parameters have changed.
  this->Modified();

  itkDebugMacro(<<"After setting parameters ");
}

// Get Parameters
template < class TRepresenter, class TScalarType,  unsigned int TDimension >
const typename AdvancedStatisticalModelTransformBase<TRepresenter,  TScalarType, TDimension>::ParametersType &
AdvancedStatisticalModelTransformBase<TRepresenter,  TScalarType, TDimension>
::GetParameters( void ) const
{
  itkDebugMacro( << "Getting parameters ");


  // Get the translation
  for(unsigned int i=0; i < this->GetNumberOfParameters(); i++)
    {
    this->m_Parameters[i] = this->m_coeff_vector[i];
    }
  itkDebugMacro(<<"After getting parameters " << this->m_Parameters );

  return this->m_Parameters;
}




// Print self
template < class TRepresenter, class TScalarType,  unsigned int TDimension >
void
AdvancedStatisticalModelTransformBase<TRepresenter,  TScalarType, TDimension>::
PrintSelf(std::ostream &os, Indent indent) const
{
	Superclass::PrintSelf(os,indent);
}




/**
 * ********************* GetJacobian ****************************
 */

template < class TRepresenter, class TScalarType,  unsigned int TDimension >
void
AdvancedStatisticalModelTransformBase<TRepresenter,  TScalarType, TDimension>
::GetJacobian(
  const InputPointType & pt,
  JacobianType & jacobian,
  NonZeroJacobianIndicesType & nonZeroJacobianIndices ) const
{
	jacobian.SetSize(OutputSpaceDimension, m_StatisticalModel->GetNumberOfPrincipalComponents());
	jacobian.Fill(0);

	const MatrixType& statModelJacobian = m_StatisticalModel->GetJacobian(pt);

	for (unsigned i = 0; i < statModelJacobian.rows(); i++) {
		for (unsigned j = 0; j <  std::min(m_usedNumberCoefficients, (unsigned) this->GetNumberOfParameters()); j++) {
			jacobian[i][j] = statModelJacobian[i][j];
		}
	}


	itkDebugMacro( << "Jacobian with MM:\n" << jacobian);
	itkDebugMacro( << "After GetMorphableModelJacobian:"
			<< "\nJacobian = \n" << jacobian);

	nonZeroJacobianIndices.resize(this->m_StatisticalModel->GetNumberOfPrincipalComponents());
	for(int i = 0; i < nonZeroJacobianIndices.size(); i++) {
		nonZeroJacobianIndices[i] = i;
	}

} // end GetJacobian()

} // namespace

#endif

