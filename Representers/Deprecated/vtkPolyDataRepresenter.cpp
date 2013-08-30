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


#ifndef __VTKPOLYDATAREPRESENTER_CPP
#define __VTKPOLYDATAREPRESENTER_CPP

#include "vtkPoints.h"
#include "statismo/HDF5Utils.h"
#include "statismo/utils.h"
#include "vtkPolyDataReader.h"
#include "vtkPolyDataWriter.h"

using statismo::VectorType;
using statismo::HDF5Utils;
using statismo::StatisticalModelException;

namespace statismo {

inline
vtkPolyDataRepresenter::vtkPolyDataRepresenter(DatasetConstPointerType reference, AlignmentType alignment)
  :
        m_alignment(alignment),
        m_pdTransform(vtkTransformPolyDataFilter::New())
{
	SetReference(reference);
}

inline
vtkPolyDataRepresenter::~vtkPolyDataRepresenter() {
	if (m_pdTransform != 0) {
		m_pdTransform->Delete();
		m_pdTransform = 0;
	}
	if (m_reference != 0) {
		m_reference->Delete();
		m_reference = 0;
	}
}

inline
void vtkPolyDataRepresenter::SetReference(const vtkPolyData* reference) {
	m_reference = vtkPolyData::New();
	   m_reference->DeepCopy(const_cast<DatasetPointerType>(reference));

	   // set the domain
	   DomainType::DomainPointsListType ptList;
	   for (unsigned i = 0; i < m_reference->GetNumberOfPoints(); i++) {
		   double* d = m_reference->GetPoint(i);
		   ptList.push_back(vtkPoint(d));
	   }
	   m_domain = DomainType(ptList);

}

inline
vtkPolyDataRepresenter*
vtkPolyDataRepresenter::Clone() const
{
	// this works since Create deep copies the reference
	return Create(m_reference, m_alignment);
}

inline
void
vtkPolyDataRepresenter::Load(const H5::CommonFG& fg) {


	std::string tmpfilename = statismo::Utils::CreateTmpName(".vtk");

	HDF5Utils::getFileFromHDF5(fg, "./reference", tmpfilename.c_str());
	SetReference(ReadDataset(tmpfilename.c_str()));
	std::remove(tmpfilename.c_str());

	m_alignment = static_cast<AlignmentType>(HDF5Utils::readInt(fg, "./alignment"));

}


inline
void
vtkPolyDataRepresenter::Save(const H5::CommonFG& fg) const {
	using namespace H5;

	std::string tmpfilename = statismo::Utils::CreateTmpName(".vtk");


	WriteDataset(tmpfilename.c_str(), this->m_reference);

	HDF5Utils::dumpFileToHDF5(tmpfilename.c_str(), fg, "./reference" );

	std::remove(tmpfilename.c_str());
	HDF5Utils::writeInt(fg, "./alignment", m_alignment);

}

inline
vtkPolyDataRepresenter::DatasetPointerType
vtkPolyDataRepresenter::DatasetToSample(DatasetConstPointerType _pd) const
{
	assert(m_reference != 0);

	vtkPolyData* reference = const_cast<vtkPolyData*>(this->m_reference);
	vtkPolyData* pd = const_cast<vtkPolyData*>(_pd);


	vtkPolyData* alignedPd  = vtkPolyData::New();

	if (m_alignment != NONE) {

		vtkLandmarkTransform* transform = vtkLandmarkTransform::New();
		// we align all the dataset to the common reference

	  transform->SetSourceLandmarks(pd->GetPoints());
	  transform->SetTargetLandmarks(m_reference->GetPoints());
	  transform->SetMode(m_alignment);

	  m_pdTransform->SetInput(pd);
	  m_pdTransform->SetTransform(transform);
	  m_pdTransform->Update();

	  // we need to shallow copy the objet to make sure it does not die with the transform
	  alignedPd->ShallowCopy(m_pdTransform->GetOutput());

	  transform->Delete();

	}
	else {
	  // no alignment needed
		alignedPd->DeepCopy(pd);
	}

	return alignedPd;
}

inline
statismo::VectorType
vtkPolyDataRepresenter::SampleToSampleVector(DatasetConstPointerType _sample) const {
	assert(m_reference != 0);

	vtkPolyData* sample = const_cast<vtkPolyData*>(_sample);

	VectorType sampleVec = VectorType::Zero(m_reference->GetNumberOfPoints() * 3);
	// TODO make this more efficient using SetVoidArray of vtk
	for (unsigned i = 0 ; i < m_reference->GetNumberOfPoints(); i++) {
		for (unsigned j = 0; j < 3; j++) {
			unsigned idx = MapPointIdToInternalIdx(i, j);
			sampleVec(idx) = sample->GetPoint(i)[j];
		}
	}
	return sampleVec;
}


inline
vtkPolyDataRepresenter::DatasetPointerType
vtkPolyDataRepresenter::SampleVectorToSample(const VectorType& sample) const
{

	assert (m_reference != 0);

	vtkPolyData* reference = const_cast<vtkPolyData*>(m_reference);
	vtkPolyData* pd = vtkPolyData::New();
	pd->DeepCopy(reference);

	vtkPoints* points = pd->GetPoints();
	for (unsigned i = 0; i < reference->GetNumberOfPoints(); i++) {
		vtkPoint pt;
		for (unsigned d = 0; d < GetDimensions(); d++) {
			unsigned idx = MapPointIdToInternalIdx(i, d);
			pt[d] = sample(idx);
		}
		points->SetPoint(i, pt.data());
	}

	return pd;
}

inline
vtkPolyDataRepresenter::ValueType
vtkPolyDataRepresenter::PointSampleFromSample(DatasetConstPointerType sample_, unsigned ptid) const {
	vtkPolyData* sample = const_cast<DatasetPointerType>(sample_);
	if (ptid >= sample->GetNumberOfPoints()) {
		throw StatisticalModelException("invalid ptid provided to PointSampleFromSample");
	}
	return vtkPoint(sample->GetPoints()->GetPoint(ptid));
}

inline
statismo::VectorType
vtkPolyDataRepresenter::PointSampleToPointSampleVector(const ValueType& v) const
{
	VectorType vec(GetDimensions());
	for (unsigned i = 0; i < GetDimensions(); i++) {
		vec(i) = v[i];
	}
	return vec;
}


inline
vtkPolyDataRepresenter::ValueType
vtkPolyDataRepresenter::PointSampleVectorToPointSample(const VectorType& v) const
{
	ValueType value;
	for (unsigned i = 0; i < GetDimensions(); i++) {
		value[i] = v(i);
	}
	return value;
}



inline
unsigned
vtkPolyDataRepresenter::GetPointIdForPoint(const PointType& pt) const {
	assert (m_reference != 0);
    return this->m_reference->FindPoint(const_cast<double*>(pt.data()));
}

inline
unsigned
vtkPolyDataRepresenter::GetNumberOfPoints() const {
	assert (m_reference != 0);

    return this->m_reference->GetNumberOfPoints();
}


inline
vtkPolyDataRepresenter::DatasetPointerType
vtkPolyDataRepresenter::ReadDataset(const std::string& filename) {
	vtkPolyData* pd = vtkPolyData::New();

    vtkPolyDataReader* reader = vtkPolyDataReader::New();
    reader->SetFileName(filename.c_str());
    reader->Update();
    if (reader->GetErrorCode() != 0) {
        throw StatisticalModelException((std::string("Could not read file ") + filename).c_str());
    }
    pd->ShallowCopy(reader->GetOutput());
    reader->Delete();
    return pd;
}

inline
void vtkPolyDataRepresenter::WriteDataset(const std::string& filename,DatasetConstPointerType pd) {
    vtkPolyDataWriter* writer = vtkPolyDataWriter::New();
    writer->SetFileName(filename.c_str());
    writer->SetInput(const_cast<vtkPolyData*>(pd));
    writer->Update();
    if (writer->GetErrorCode() != 0) {
        throw StatisticalModelException((std::string("Could not read file ") + filename).c_str());
    }
    writer->Delete();
}



} // namespace statismo

#endif // __VTKPOLYDATAREPRESENTER_CPP
