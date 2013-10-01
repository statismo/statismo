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



#ifndef VTKSTRUCTUREDPOINTSREPRESENTER_H_
#define VTKSTRUCTUREDPOINTSREPRESENTER_H_

#include "vtkStructuredPoints.h"
#include "statismo/Representer.h"
#include "vtkPoint.h"
#include "vtkPixel.h"
#include "vtkHelper.h"
#include "statismo/CommonTypes.h"
#include "statismo/Domain.h"
#include "vtkSmartPointer.h"

#include <H5Cpp.h>

/**
 * \brief Representer class for vtkStructuredPoints of arbitrary scalar type and dimension
 * \sa Representer
 */

namespace statismo {


template <>
struct RepresenterTraits<vtkStructuredPoints> {
	typedef vtkStructuredPoints* DatasetPointerType;
	typedef const vtkStructuredPoints* DatasetConstPointerType;

	typedef vtkPoint PointType;
	typedef vtkNDPixel ValueType;


	static void DeleteDataset(DatasetPointerType d) {
		d->Delete();
	};
    ///@}


};


class vtkStructuredPointsRepresenter  : public Representer<vtkStructuredPoints> {
public:

	static vtkStructuredPointsRepresenter* Create() { return new vtkStructuredPointsRepresenter(); }
	static vtkStructuredPointsRepresenter* Create(const vtkStructuredPoints* reference) { return new vtkStructuredPointsRepresenter(reference); }

	void Load(const H5::CommonFG& fg);
	vtkStructuredPointsRepresenter* Clone() const;


	virtual ~vtkStructuredPointsRepresenter();
	void Delete() const { delete this; }



	unsigned GetDimensions() const { return  m_reference->GetDataDimension(); }
	const DomainType& GetDomain() const  { return m_domain; }

	std::string GetName() const { return "vtkStructuredPointsRepresenter"; }

	const vtkStructuredPoints* GetReference() const { return m_reference; }

	statismo::VectorType PointToVector(const PointType& pt) const;

	DatasetPointerType DatasetToSample(DatasetConstPointerType ds) const;
	statismo::VectorType SampleToSampleVector(DatasetConstPointerType sample) const;
	DatasetPointerType SampleVectorToSample(const statismo::VectorType& sample) const;


	ValueType PointSampleFromSample(DatasetConstPointerType sample, unsigned ptid) const;
	statismo::VectorType PointSampleToPointSampleVector(const ValueType& v) const;
	ValueType PointSampleVectorToPointSample(const statismo::VectorType& samplePoint) const;

	unsigned GetPointIdForPoint(const PointType& pt) const;

	unsigned GetNumberOfPoints() const;
	void Save(const H5::CommonFG& fg) const;

	unsigned GetNumberOfPoints(DatasetPointerType  reference) const;


private:

	vtkStructuredPointsRepresenter(const vtkStructuredPoints* reference);
	vtkStructuredPointsRepresenter() : m_reference(0) {}

    static DatasetPointerType ReadDataset(const std::string& filename);
	static void WriteDataset(const std::string& filename, DatasetConstPointerType sp);

	void SetReference(const vtkStructuredPoints* reference);

	vtkStructuredPoints* m_reference;
	DomainType m_domain;
};

} // namespace statismo

#include "vtkStructuredPointsRepresenter.txx"


#endif /* VTKSTRUCTUREDPOINTSREPRESENTER_H_ */
