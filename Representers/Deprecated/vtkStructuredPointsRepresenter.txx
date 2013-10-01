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


#include "vtkStructuredPointsRepresenter.h"
#include "vtkStructuredPointsReader.h"
#include "vtkStructuredPointsWriter.h"
#include "vtkPointData.h"
#include "vtkDataArray.h"
#include "statismo/HDF5Utils.h"
#include "statismo/utils.h"

using statismo::VectorType;
using statismo::HDF5Utils;
using statismo::StatisticalModelException;

namespace statismo {

vtkStructuredPointsRepresenter::vtkStructuredPointsRepresenter(DatasetConstPointerType reference)
  : m_reference(vtkStructuredPoints::New())
{
	SetReference(reference);
}


vtkStructuredPointsRepresenter::~vtkStructuredPointsRepresenter() {

	if (m_reference != 0) {
		m_reference->Delete();
		m_reference = 0;
	}
}

vtkStructuredPointsRepresenter*
vtkStructuredPointsRepresenter::Clone() const
{
	// this works since Create deep copies the reference
	return Create(m_reference);
}

void
vtkStructuredPointsRepresenter::Load(const H5::CommonFG& fg) {

	std::string tmpfilename = statismo::Utils::CreateTmpName(".vtk");

	statismo::HDF5Utils::getFileFromHDF5(fg, "./reference", tmpfilename.c_str());
	DatasetConstPointerType reference = ReadDataset(tmpfilename.c_str());

	std::remove(tmpfilename.c_str());

	SetReference(reference);
}

template <class TPrecision, unsigned Dimensions>
statismo::VectorType
vtkStructuredPointsRepresenter<TPrecision, Dimensions>::PointToVector(const PointType& pt) const {
        // a vtk point is always 3 dimensional
        VectorType v(3);
        for (unsigned i = 0; i < 3; i++) {
                v(i) = pt[i];
        }
        return v;
}

typename vtkStructuredPointsRepresenter::DatasetPointerType
vtkStructuredPointsRepresenter::DatasetToSample(DatasetConstPointerType dataset) const
{
	// for this representer, a dataset is always the same as a sample
	vtkStructuredPoints* clone = vtkStructuredPoints::New();
	clone->DeepCopy(const_cast<vtkStructuredPoints*>(dataset));


	if (const_cast<vtkStructuredPoints*>(m_reference)->GetNumberOfPoints() != const_cast<vtkStructuredPoints*>(dataset)->GetNumberOfPoints()) {
		throw StatisticalModelException("The dataset need to have the same number of points as the reference");
	}


	return clone;
}

VectorType
vtkStructuredPointsRepresenter::SampleToSampleVector(DatasetConstPointerType _sp) const
{

	vtkStructuredPoints* reference = const_cast<vtkStructuredPoints*>(this->m_reference);
	vtkStructuredPoints* sp = const_cast<vtkStructuredPoints*>(_sp);

	if (reference->GetNumberOfPoints() != sp->GetNumberOfPoints()) {
		throw StatisticalModelException("The sample does not have the correct number of points");
	}

	VectorType sample = VectorType::Zero(m_reference->GetNumberOfPoints() * GetDimensions());

	vtkDataArray* scalars = sp->GetPointData()->GetScalars();
	// TODO make this more efficient using SetVoidArray of vtk
	for (unsigned i = 0 ; i < m_reference->GetNumberOfPoints(); i++) {

		double val[GetDimensions()];
		scalars->GetTuple(i, val);
		for (unsigned d = 0; d < GetDimensions(); d++) {
			unsigned idx = MapPointIdToInternalIdx(i, d);
			sample(idx) = val[d];
		}
	}
	return sample;
}



typename vtkStructuredPointsRepresenter::DatasetPointerType
vtkStructuredPointsRepresenter::SampleVectorToSample(const VectorType& sample) const
{
	vtkStructuredPoints* sp = vtkStructuredPoints::New();
	vtkStructuredPoints* reference = const_cast<vtkStructuredPoints*>(m_reference);
	sp->DeepCopy(reference);

	vtkDataArray* scalars = sp->GetPointData()->GetScalars();
	for (unsigned i = 0; i < GetNumberOfPoints(); i++) {
		double val[GetDimensions()];
		for (unsigned d = 0; d < GetDimensions(); d++) {
			unsigned idx = MapPointIdToInternalIdx(i, d);
			val[d] = sample(idx);
		}
		scalars->SetTuple(i, val);
	}
	sp->GetPointData()->SetScalars(scalars);
	return sp;
}


typename vtkStructuredPointsRepresenter::ValueType
vtkStructuredPointsRepresenter::PointSampleFromSample(DatasetConstPointerType sample_, unsigned ptid) const {
	DatasetPointerType sample = const_cast<DatasetPointerType>(sample_);
	if (ptid >= sample->GetNumberOfPoints()) {
		throw StatisticalModelException("invalid ptid provided to PointSampleFromSample");
	}

	double doubleVal[GetDimensions()];
	sample->GetPointData()->GetScalars()->GetTuple(ptid, doubleVal);
	ValueType val(GetDimensions());

	// vtk returns double. We need to convert it to whatever precision we are using in NDPixel
	for (unsigned i = 0; i < GetDimensions(); i++) {
		val[i] = static_cast<double>(doubleVal[i]);
	}
	return val;
}


statismo::VectorType
vtkStructuredPointsRepresenter::PointSampleToPointSampleVector(const ValueType& v) const
{
	VectorType vec(GetDimensions());
	for (unsigned i = 0; i < GetDimensions(); i++) {
		vec[i] = const_cast<ValueType&>(v)[i];
		vec[i] = v[i];
	}
	return vec;
}

typename vtkStructuredPointsRepresenter::ValueType
vtkStructuredPointsRepresenter::PointSampleVectorToPointSample(const VectorType& pointSample) const
{
	ValueType value(GetDimensions());

	for (unsigned i = 0; i < GetDimensions(); i++)
		value[i] = pointSample[i];

	return value;
}




void
vtkStructuredPointsRepresenter::Save(const H5::CommonFG& fg) const {
	using namespace H5;

	std::string tmpfilename = statismo::Utils::CreateTmpName(".vtk");

	WriteDataset(tmpfilename.c_str(), this->m_reference);

	statismo::HDF5Utils::dumpFileToHDF5(tmpfilename.c_str(), fg, "./reference" );

	std::remove(tmpfilename.c_str());

}

unsigned
vtkStructuredPointsRepresenter::GetNumberOfPoints() const {
	return GetNumberOfPoints(this->m_reference);
}


typename vtkStructuredPointsRepresenter::DatasetPointerType
vtkStructuredPointsRepresenter::ReadDataset(const std::string& filename) {

	vtkStructuredPoints* sp = vtkStructuredPoints::New();

    vtkStructuredPointsReader* reader = vtkStructuredPointsReader::New();
    reader->SetFileName(filename.c_str());
    reader->Update();
    if (reader->GetErrorCode() != 0) {
        throw StatisticalModelException((std::string("Could not read file ") + filename).c_str());
    }

    sp->DeepCopy(reader->GetOutput());

    reader->Delete();
    return sp;
}

void vtkStructuredPointsRepresenter::WriteDataset(const std::string& filename, const vtkStructuredPoints* sp) {
    vtkStructuredPointsWriter* writer = vtkStructuredPointsWriter::New();
    writer->SetFileName(filename.c_str());
    writer->SetInput(const_cast<vtkStructuredPoints*>(sp));
    if (writer->GetErrorCode() != 0) {
        throw StatisticalModelException((std::string("Could not write file ") + filename).c_str());
    }
    writer->Update();

    writer->Delete();
}

unsigned
vtkStructuredPointsRepresenter::GetPointIdForPoint(const PointType& pt) const {
	assert (m_reference != 0);
    return this->m_reference->FindPoint(const_cast<double*>(pt.data()));
}


unsigned vtkStructuredPointsRepresenter::GetNumberOfPoints(vtkStructuredPoints* reference) const {
    return reference->GetNumberOfPoints();
}


void vtkStructuredPointsRepresenter::SetReference(const vtkStructuredPoints* reference) {
	m_reference->DeepCopy(const_cast<vtkStructuredPoints*>(reference));
	// set the domain
   DomainType::DomainPointsListType ptList;
   for (unsigned i = 0; i < m_reference->GetNumberOfPoints(); i++) {
	   double* d = m_reference->GetPoint(i);
	   ptList.push_back(vtkPoint(d));
   }
	m_domain = DomainType(ptList);
}

} // namespace statismo
