/*
 * This file is part of the statismo library.
 *
 * Author: Marcel Luethi (marcel.luethi@unibas.ch)
 *
 * Copyright (c) 2011 University of Basel
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the project's author nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __itkStatisticalModelTransform_txx
#define __itkStatisticalModelTransform_txx

#include "itkStatisticalModelTransform.h"


namespace itk
{

/*!
 * Constructor with default arguments.
 */
template < class TRepresenter, class TScalarType,  unsigned int TDimension >
StatisticalModelTransform<TRepresenter,  TScalarType, TDimension>
::StatisticalModelTransform() :
	Superclass(OutputSpaceDimension, 0),
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
StatisticalModelTransform<TRepresenter,  TScalarType, TDimension>
::SetStatisticalModel(const StatisticalModelType* model)
{
	itkDebugMacro( << "Setting statistical model ");
	m_StatisticalModel = model;


	this->m_Parameters.SetSize(model->GetNumberOfPrincipalComponents());
	this->m_Parameters.Fill(0.0);

	this->m_coeff_vector.set_size(model->GetNumberOfPrincipalComponents());

	this->m_Jacobian.SetSize(OutputSpaceDimension, model->GetNumberOfPrincipalComponents());
	this->m_Jacobian.Fill(0);
}

template < class TRepresenter, class TScalarType,  unsigned int TDimension >
typename StatisticalModelTransform<TRepresenter,  TScalarType, TDimension>::StatisticalModelType::ConstPointer
StatisticalModelTransform<TRepresenter,  TScalarType, TDimension>
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
StatisticalModelTransform<TRepresenter,  TScalarType, TDimension>
::SetIdentity( )
{
	itkDebugMacro( << "Setting Identity");

	for (unsigned i = 0; i  < this->GetNumberOfParameters(); i++)
		this->m_coeff_vector[i] = 0;


	this->Modified();
}

template < class TRepresenter, class TScalarType,  unsigned int TDimension >
void
StatisticalModelTransform<TRepresenter,  TScalarType, TDimension>
::SetParameters( const ParametersType & parameters )
{
  itkDebugMacro( << "Setting parameters " << parameters );

  // Set angle
  for(unsigned int i=0; i < std::min(m_usedNumberCoefficients, this->GetNumberOfParameters()); i++)
    {
	m_coeff_vector[i] = parameters[i];
    }
  for (unsigned int i = std::min(m_usedNumberCoefficients, this->GetNumberOfParameters()); i <  this->GetNumberOfParameters(); i++) {
	  m_coeff_vector[i] = 0;
  }

  // Modified is always called since we just have a pointer to the
  // parameters and cannot know if the parameters have changed.
  this->Modified();

  itkDebugMacro(<<"After setting parameters ");
}





// Get Parameters
template < class TRepresenter, class TScalarType,  unsigned int TDimension >
const typename StatisticalModelTransform<TRepresenter,  TScalarType, TDimension>::ParametersType &
StatisticalModelTransform<TRepresenter,  TScalarType, TDimension>
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





/* Transform*/
template < class TRepresenter, class TScalarType,  unsigned int TDimension >
typename StatisticalModelTransform<TRepresenter,  TScalarType, TDimension>::OutputPointType
StatisticalModelTransform<TRepresenter,  TScalarType, TDimension>
::TransformPoint(const InputPointType &pt) const
{
	itkDebugMacro( << "TransformMorphableModelPoint()" );

	typename RepresenterType::ValueType d;
	try {
		d = m_StatisticalModel->DrawInstanceAtPoint(this->m_coeff_vector, pt);
	} catch (ExceptionObject &e) {
		std::cout << "exception occured at point " << pt << std::endl;
		std::cout << "message " << e.what() << std::endl;
	}
	return pt + d;
}






template < class TRepresenter, class TScalarType,  unsigned int TDimension >
const typename StatisticalModelTransform<TRepresenter,  TScalarType, TDimension>::JacobianType &
StatisticalModelTransform<TRepresenter,  TScalarType, TDimension>::
GetJacobian(const InputPointType &pt) const
{

	this->m_Jacobian.Fill(0.0);


	const MatrixType& statModelJacobian = m_StatisticalModel->GetJacobian(pt);

	for (unsigned i = 0; i < statModelJacobian.rows(); i++) {
		for (unsigned j = 0; j <  std::min(m_usedNumberCoefficients, this->GetNumberOfParameters()); j++) {
			this->m_Jacobian[i][j] = statModelJacobian[i][j];
		}
	}


	itkDebugMacro( << "Jacobian with MM:\n" << this->m_Jacobian);
	itkDebugMacro( << "After GetMorphableModelJacobian:"
			<< "\nJacobian = \n" << this->m_Jacobian);
	return this->m_Jacobian;

}


// Print self
template < class TRepresenter, class TScalarType,  unsigned int TDimension >
void
StatisticalModelTransform<TRepresenter,  TScalarType, TDimension>::
PrintSelf(std::ostream &os, Indent indent) const
{
	Superclass::PrintSelf(os,indent);
}

} // namespace

#endif
