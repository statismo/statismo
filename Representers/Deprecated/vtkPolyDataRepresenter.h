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


#ifndef VTKPOLYDATAREPRESENTER_H_
#define VTKPOLYDATAREPRESENTER_H_

#include "statismo/Representer.h"
#include "vtkPolyData.h"
#include "vtkLandmarkTransform.h"
#include "vtkTransform.h"
#include "vtkTransformPolyDataFilter.h"
#include "vtkHelper.h"
#include "statismo/CommonTypes.h"
#include "statismo/Domain.h"
#include "vtkSmartPointer.h"
#include <H5Cpp.h>



/**
 * \brief A representer for vtkPolyData using Procrustes alignment to align the datasets
 *
 * This class provides a specialization of the Representer for the type vtkPolyData.
 * Procrustes is used to align the given datasets with the reference.
 * The user can choose between Rigid, Similarity and Affine alignment.
 *
 * Hint: In order to use GPA alignment, simply set the Procrustes Mean as the reference.
 *
 * Warning: This class does currently not provide any registration, which implies
 * that the dataset that are read by the class need to be aligned.
 *
 * See Representer for more details about representer classes
 * \sa Representer
 */

namespace statismo {

template <>
struct RepresenterTraits<vtkPolyData> {
	typedef vtkPolyData* DatasetPointerType;
	typedef const vtkPolyData* DatasetConstPointerType;

	typedef vtkPoint PointType;
	typedef vtkPoint ValueType;

	static void DeleteDataset(DatasetPointerType d) {
		d->Delete();
	};
    ///@}


};



class vtkPolyDataRepresenter  : public Representer<vtkPolyData>{
public:

	/// The type of the data set to be used

	enum AlignmentType {
	  NONE=999, // something that VTK does not define
	  RIGID=VTK_LANDMARK_RIGIDBODY,
	  SIMILARITY=VTK_LANDMARK_SIMILARITY,
	  AFFINE=VTK_LANDMARK_AFFINE
	};

	static vtkPolyDataRepresenter* Create() { return new vtkPolyDataRepresenter(); }

	static vtkPolyDataRepresenter* Create(DatasetConstPointerType reference, AlignmentType alignment) {
		return new vtkPolyDataRepresenter(reference, alignment);
	}

	void Load(const H5::CommonFG& fg);

	vtkPolyDataRepresenter* Clone() const;
	void Delete() const { delete this; }

	virtual ~vtkPolyDataRepresenter();


	std::string GetName() const { return "vtkPolyDataRepresenter"; }
	unsigned GetDimensions() const { return 3; }

	const DomainType& GetDomain() const { return m_domain; }

	AlignmentType GetAlignment() const { return m_alignment; }

	DatasetConstPointerType GetReference() const { return m_reference; }

	DatasetPointerType DatasetToSample(DatasetConstPointerType ds) const;
	statismo::VectorType SampleToSampleVector(DatasetConstPointerType sample) const;
	DatasetPointerType SampleVectorToSample(const statismo::VectorType& sample) const;

	ValueType PointSampleFromSample(DatasetConstPointerType sample, unsigned ptid) const;
	statismo::VectorType PointSampleToPointSampleVector(const ValueType& v) const;
	ValueType PointSampleVectorToPointSample(const statismo::VectorType& pointSample) const;


	void Save(const H5::CommonFG& fg) const;
	unsigned GetNumberOfPoints() const;
	unsigned GetPointIdForPoint(const PointType& point) const;




private:

	vtkPolyDataRepresenter() : m_reference(0), m_pdTransform(0), m_alignment(NONE) {}

	vtkPolyDataRepresenter(const std::string& reference, AlignmentType alignment);
	vtkPolyDataRepresenter(const DatasetConstPointerType reference, AlignmentType alignment);
	vtkPolyDataRepresenter(const vtkPolyDataRepresenter& orig);
	vtkPolyDataRepresenter& operator=(const vtkPolyDataRepresenter& rhs);

	void SetReference(const vtkPolyData* reference);

	static DatasetPointerType ReadDataset(const std::string& filename);
	static void WriteDataset(const std::string& filename, DatasetConstPointerType pd) ;


	DatasetPointerType m_reference;

	vtkTransformPolyDataFilter* m_pdTransform;
	AlignmentType m_alignment;
	DomainType m_domain;
};

} // namespace statismo

#include "vtkPolyDataRepresenter.cpp"

#endif /* VTKPOLYDATAREPRESENTER_H_ */
