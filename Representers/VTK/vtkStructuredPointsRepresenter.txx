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


#ifdef _WIN32
#include <windows.h>
#include <tchar.h>
#endif

#include "vtkStructuredPointsRepresenter.h"
#include "vtkPointData.h"
#include "vtkStructuredPointsReader.h"
#include "vtkStructuredPointsWriter.h"
#include "statismo/HDF5Utils.h"

using statismo::VectorType;
using statismo::HDF5Utils;
using statismo::StatisticalModelException;



template <class TPrecision, unsigned Dimensions>
vtkStructuredPointsRepresenter<TPrecision, Dimensions>::vtkStructuredPointsRepresenter(DatasetConstPointerType reference)
  : m_reference(NewDataset())
{
	m_reference->DeepCopy(const_cast<vtkStructuredPoints*>(reference));
}

template <class TPrecision, unsigned Dimensions>
vtkStructuredPointsRepresenter<TPrecision, Dimensions>::vtkStructuredPointsRepresenter(const std::string& referenceFilename)
  : m_reference(ReadDataset(referenceFilename))
{}


template <class TPrecision, unsigned Dimensions>
vtkStructuredPointsRepresenter<TPrecision, Dimensions>::~vtkStructuredPointsRepresenter() {

	if (m_reference != 0) {
		m_reference->Delete();
		m_reference = 0;
	}
}

template <class TPrecision, unsigned Dimensions>
vtkStructuredPointsRepresenter<TPrecision, Dimensions>*
vtkStructuredPointsRepresenter<TPrecision, Dimensions>::Clone() const
{
	// this works since Create deep copies the reference
	return Create(m_reference);
}

template <class TPrecision, unsigned Dimensions>
vtkStructuredPointsRepresenter<TPrecision, Dimensions>*
vtkStructuredPointsRepresenter<TPrecision, Dimensions>::Load(const H5::CommonFG& fg) {

#ifdef _WIN32
	std::string tmpDirectoryName;
	TCHAR szTempFileName[MAX_PATH];
	DWORD dwRetVal = 0;
	//  Gets the temp path env string (no guarantee it's a valid path).
    dwRetVal = GetTempPath(MAX_PATH,          // length of the buffer
                           szTempFileName); // buffer for path
	tmpDirectoryName.assign(szTempFileName);
	std::string tmpfilename = tmpDirectoryName + "/" + tmpnam(0);
#else
	std::string tmpfilename = tmpnam(0);
#endif


	statismo::HDF5Utils::getFileFromHDF5(fg, "./reference", tmpfilename.c_str());
	DatasetConstPointerType reference = ReadDataset(tmpfilename.c_str());

	std::remove(tmpfilename.c_str());

	return Create(reference);
}



template <class TPrecision, unsigned Dimensions>
typename vtkStructuredPointsRepresenter<TPrecision, Dimensions>::DatasetPointerType
vtkStructuredPointsRepresenter<TPrecision, Dimensions>::DatasetToSample(DatasetConstPointerType dataset, DatasetInfo* notUsed) const
{
	// for this representer, a dataset is always the same as a sample
	vtkStructuredPoints* clone = vtkStructuredPoints::New();
	clone->DeepCopy(const_cast<vtkStructuredPoints*>(dataset));
	return clone;
}

template <class TPrecision, unsigned Dimensions>
VectorType
vtkStructuredPointsRepresenter<TPrecision, Dimensions>::SampleToSampleVector(DatasetConstPointerType _sp) const
{

	vtkStructuredPoints* reference = const_cast<vtkStructuredPoints*>(this->m_reference);
	vtkStructuredPoints* sp = const_cast<vtkStructuredPoints*>(_sp);
	VectorType sample = VectorType::Zero(m_reference->GetNumberOfPoints() * Dimensions);

	vtkDataArray* scalars = sp->GetPointData()->GetScalars();
	// TODO make this more efficient using SetVoidArray of vtk
	for (unsigned i = 0 ; i < m_reference->GetNumberOfPoints(); i++) {

		double val[Dimensions];
		scalars->GetTuple(i, val);
		for (unsigned d = 0; d < Dimensions; d++) {
			unsigned idx = MapPointIdToInternalIdx(i, d);
			sample(idx) = val[d];
		}
	}
	return sample;
}



template <class TPrecision, unsigned Dimensions>
typename vtkStructuredPointsRepresenter<TPrecision, Dimensions>::DatasetPointerType
vtkStructuredPointsRepresenter<TPrecision, Dimensions>::SampleVectorToSample(const VectorType& sample) const
{
	vtkStructuredPoints* sp = vtkStructuredPoints::New();
	vtkStructuredPoints* reference = const_cast<vtkStructuredPoints*>(m_reference);
	sp->DeepCopy(reference);

	vtkDataArray* scalars = sp->GetPointData()->GetScalars();
	for (unsigned i = 0; i < GetNumberOfPoints(); i++) {
		double val[Dimensions];
		for (unsigned d = 0; d < Dimensions; d++) {
			unsigned idx = MapPointIdToInternalIdx(i, d);
			val[d] = sample(idx);
		}
		scalars->SetTuple(i, val);
	}
	sp->GetPointData()->SetScalars(scalars);
	return sp;
}

template <class TPrecision, unsigned Dimensions>
statismo::VectorType
vtkStructuredPointsRepresenter<TPrecision, Dimensions>::PointSampleToPointSampleVector(const ValueType& v) const
{
	VectorType vec(Dimensions);
	for (unsigned i = 0; i < Dimensions; i++) {
		vec[i] = const_cast<ValueType&>(v)[i];
		vec[i] = v[i];
	}
	return vec;
}

template <class TPrecision, unsigned Dimensions>
typename vtkStructuredPointsRepresenter<TPrecision, Dimensions>::ValueType
vtkStructuredPointsRepresenter<TPrecision, Dimensions>::PointSampleVectorToPointSample(const VectorType& pointSample) const
{
	ValueType value;

	for (unsigned i = 0; i < Dimensions; i++)
		value[i] = pointSample[i];

	return value;
}




template <class TPrecision, unsigned Dimensions>
void
vtkStructuredPointsRepresenter<TPrecision, Dimensions>::Save(const H5::CommonFG& fg) const {
	using namespace H5;

#ifdef _WIN32
	std::string tmpDirectoryName;
	TCHAR szTempFileName[MAX_PATH];
	DWORD dwRetVal = 0;
	//  Gets the temp path env string (no guarantee it's a valid path).
    dwRetVal = GetTempPath(MAX_PATH,          // length of the buffer
                           szTempFileName); // buffer for path
	tmpDirectoryName.assign(szTempFileName);
	std::string tmpfilename = tmpDirectoryName + "/" + tmpnam(0);
#else
	std::string tmpfilename = tmpnam(0);
#endif


	WriteDataset(tmpfilename.c_str(), this->m_reference);

	statismo::HDF5Utils::dumpFileToHDF5(tmpfilename.c_str(), fg, "./reference" );

	std::remove(tmpfilename.c_str());

}

template <class TPrecision, unsigned Dimensions>
unsigned
vtkStructuredPointsRepresenter<TPrecision, Dimensions>::GetNumberOfPoints() const {
	return GetNumberOfPoints(this->m_reference);
}



template <class TPrecision, unsigned Dimensions>
typename vtkStructuredPointsRepresenter<TPrecision, Dimensions>::DatasetPointerType
vtkStructuredPointsRepresenter<TPrecision, Dimensions>::ReadDataset(const std::string& filename) {

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

template <class TPrecision, unsigned Dimensions>
void vtkStructuredPointsRepresenter<TPrecision, Dimensions>::WriteDataset(const std::string& filename, const vtkStructuredPoints* sp) {
    vtkStructuredPointsWriter* writer = vtkStructuredPointsWriter::New();
    writer->SetFileName(filename.c_str());
    writer->SetInput(const_cast<vtkStructuredPoints*>(sp));
    if (writer->GetErrorCode() != 0) {
        throw StatisticalModelException((std::string("Could not write file ") + filename).c_str());
    }
    writer->Update();

    writer->Delete();
}

template <class TPrecision, unsigned Dimensions>
unsigned
vtkStructuredPointsRepresenter<TPrecision, Dimensions>::GetPointIdForPoint(const PointType& pt) const {
	assert (m_reference != 0);
    return this->m_reference->FindPoint(const_cast<double*>(pt.data()));
}

template <class TPrecision, unsigned Dimensions>
vtkStructuredPoints* vtkStructuredPointsRepresenter<TPrecision, Dimensions>::NewDataset() {
    return vtkStructuredPoints::New();
}

template <class TPrecision, unsigned Dimensions>
void vtkStructuredPointsRepresenter<TPrecision, Dimensions>::DeleteDataset(vtkStructuredPoints* d) {
	d->Delete();
}

template <class TPrecision, unsigned Dimensions>
unsigned vtkStructuredPointsRepresenter<TPrecision, Dimensions>::GetNumberOfPoints(vtkStructuredPoints* reference) {
    return reference->GetNumberOfPoints();
}
