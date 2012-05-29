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


#include "itkIndex.h"
#include "itkPoint.h"
#include "itkVector.h"
#include "statismo/HDF5Utils.h"
#include "statismo/utils.h"
#include <iostream>
#include "itkMeshFileReader.h"
#include "itkMeshFileWriter.h"
#include "itkTransformMeshFilter.h"
#include "itkIdentityTransform.h"




namespace itk {

using statismo::VectorType;
using statismo::HDF5Utils;
using statismo::StatisticalModelException;


template <class TPixel, unsigned MeshDimension>
MeshRepresenter<TPixel, MeshDimension>::MeshRepresenter()
  : m_reference(0)
{
}
template <class TPixel, unsigned MeshDimension>
MeshRepresenter<TPixel, MeshDimension>::~MeshRepresenter() {
}

template <class TPixel, unsigned MeshDimension>
MeshRepresenter<TPixel, MeshDimension>*
MeshRepresenter<TPixel, MeshDimension>::Clone() const {

	MeshRepresenter* clone = new MeshRepresenter();
	clone->Register();

	typename MeshType::Pointer clonedReference = cloneMesh(m_reference);
	clone->SetReference(clonedReference);
	return clone;
}



template <class TPixel, unsigned MeshDimension>
MeshRepresenter<TPixel, MeshDimension>*
MeshRepresenter<TPixel, MeshDimension>::Load(const H5::CommonFG& fg) {

	MeshRepresenter* newInstance = new MeshRepresenter();
	newInstance->Register();

	std::string tmpfilename = statismo::Utils::CreateTmpName(".vtk");
	HDF5Utils::getFileFromHDF5(fg, "./reference", tmpfilename.c_str());

	newInstance->SetReference(ReadDataset(tmpfilename.c_str()));
	std::remove(tmpfilename.c_str());
	return newInstance;
}




template <class TPixel, unsigned MeshDimension>
void
MeshRepresenter<TPixel, MeshDimension>::SetReference(DatasetPointerType reference) {
	m_reference = reference;

	// We create a list of poitns for the domain.
	// Furthermore, we cache for all the points of the reference, as these are the most likely ones
	// we have to look up later.
	typename DomainType::DomainPointsListType domainPointList;

	typename PointsContainerType::Pointer points = m_reference->GetPoints();
	typename PointsContainerType::Iterator pointIterator= points->Begin();
	unsigned id = 0;
	while( pointIterator != points->End() ) {
		domainPointList.push_back(pointIterator.Value());
		m_pointCache.insert(std::pair<PointType, unsigned>(pointIterator.Value(), id));
		++pointIterator;
		++id;
	}
	m_domain = DomainType(domainPointList);

}



template <class TPixel, unsigned MeshDimension>
typename MeshRepresenter<TPixel, MeshDimension>::DatasetPointerType
MeshRepresenter<TPixel, MeshDimension>::DatasetToSample(MeshType* ds, DatasetInfo* notUsed) const
{
	// we don't do any alignment, but simply return a clone of the dataset
	return cloneMesh(ds);
}

template <class TPixel, unsigned MeshDimension>
VectorType
MeshRepresenter<TPixel, MeshDimension>::SampleToSampleVector(MeshType* mesh) const
{
	VectorType sample(GetNumberOfPoints() * GetDimensions());

	typename PointsContainerType::Pointer points = mesh->GetPoints();

	typename PointsContainerType::Iterator pointIterator= points->Begin();
	unsigned id = 0;
	while( pointIterator != points->End() ) {
		for (unsigned d = 0; d < GetDimensions(); d++) {
			unsigned idx = MapPointIdToInternalIdx(id, d);
			sample[idx] = pointIterator.Value()[d];
		}
		++pointIterator;
		++id;
	}
	return sample;
}



template <class TPixel, unsigned MeshDimension>
typename MeshRepresenter<TPixel, MeshDimension>::DatasetPointerType
MeshRepresenter<TPixel, MeshDimension>::SampleVectorToSample(const VectorType& sample) const
{
	typename MeshType::Pointer mesh = cloneMesh(m_reference);
	typename PointsContainerType::Pointer points = mesh->GetPoints();
	typename PointsContainerType::Iterator pointsIterator = points->Begin();

	unsigned ptId = 0;
	while( pointsIterator != points->End() ) {
		ValueType v;
		for (unsigned d = 0; d < GetDimensions(); d++) {
			unsigned idx = this->MapPointIdToInternalIdx(ptId, d);
			v[d] = sample[idx];
		}
		mesh->SetPoint(ptId, v);

		++ptId;
		++pointsIterator;
	}
	return mesh;
}


template <class TPixel, unsigned MeshDimension>
typename MeshRepresenter<TPixel, MeshDimension>::ValueType
MeshRepresenter<TPixel, MeshDimension>::PointSampleVectorToPointSample(const VectorType& pointSample) const
{
	ValueType value;
	for (unsigned d = 0; d < GetDimensions(); d++) {
		value[d] = pointSample[d];
	}
	return value;
}
template <class TPixel, unsigned MeshDimension>
statismo::VectorType
MeshRepresenter<TPixel, MeshDimension>::PointSampleToPointSampleVector(const ValueType& v) const
{
	VectorType vec(GetDimensions());
	for (unsigned d = 0; d < GetDimensions(); d++) {
		vec[d] = v[d];
	}
	return vec;
}


template <class TPixel, unsigned MeshDimension>
void
MeshRepresenter<TPixel, MeshDimension>::Save(const H5::CommonFG& fg) const {
	using namespace H5;

	std::string tmpfilename = statismo::Utils::CreateTmpName(".vtk");
	WriteDataset(tmpfilename.c_str(), (DatasetConstPointerType)this->m_reference);
	HDF5Utils::dumpFileToHDF5(tmpfilename.c_str(), fg, "./reference" );
	std::remove(tmpfilename.c_str());

}


template <class TPixel, unsigned MeshDimension>
unsigned
MeshRepresenter<TPixel, MeshDimension>::GetNumberOfPoints() const {
	return this->m_reference->GetNumberOfPoints();
}


template <class TPixel, unsigned MeshDimension>
unsigned
MeshRepresenter<TPixel, MeshDimension>::GetPointIdForPoint(const PointType& pt) const {
	int ptId = -1;

	// check whether the point is cached, otherwise look for it
	typename PointCacheType::const_iterator got = m_pointCache.find (pt);
	if (got == m_pointCache.end()) {
		ptId = FindClosestPoint(m_reference, pt);
		m_pointCache.insert(std::pair<PointType, unsigned>(pt, ptId));
	}
	else {
		ptId = got->second;
	}
	assert(ptId != -1);
	return static_cast<unsigned>(ptId);
}


template <class TPixel, unsigned MeshDimension>
typename MeshRepresenter<TPixel, MeshDimension>::DatasetPointerType
MeshRepresenter<TPixel, MeshDimension>::ReadDataset(const char* filename) {
    typename itk::MeshFileReader<MeshType>::Pointer reader = itk::MeshFileReader<MeshType>::New();
    reader->SetFileName(filename);
    try {
        reader->Update();
    }
    catch (itk::MeshFileReaderException& e) {
        throw StatisticalModelException((std::string("Could not read file ") + filename).c_str());
    }

    return reader->GetOutput();
}

template <class TPixel, unsigned MeshDimension>
void MeshRepresenter<TPixel, MeshDimension>::WriteDataset(const char* filename, const MeshType* mesh) {
    typename itk::MeshFileWriter<MeshType>::Pointer writer =itk::MeshFileWriter<MeshType>::New();
    writer->SetFileName(filename);
    writer->SetInput(mesh);
    try {
		writer->Update();
    }
    catch (itk::MeshFileWriterException& e) {
        throw StatisticalModelException((std::string("Could not write file ") + filename).c_str());
    }

}


template <class TPixel, unsigned MeshDimension>
typename  MeshRepresenter<TPixel, MeshDimension>::DatasetPointerType
MeshRepresenter<TPixel, MeshDimension>::cloneMesh(const MeshType* mesh) const {

	// cloning is cumbersome - therefore we let itk do the job for, and use perform a
	// Mesh transform using the identity transform. This should result in a perfect clone.

	typedef itk::IdentityTransform<TPixel, MeshDimension> IdentityTransformType;
	typedef itk::TransformMeshFilter<MeshType, MeshType, IdentityTransformType> TransformMeshFilterType;

	typename TransformMeshFilterType::Pointer tf = TransformMeshFilterType::New();
	tf->SetInput(mesh);
	typename IdentityTransformType::Pointer idTrans = IdentityTransformType::New();
	tf->SetTransform(idTrans);
	tf->Update();

	typename MeshType::Pointer clone = tf->GetOutput();
	clone->DisconnectPipeline();
	return clone;
}

template <class TPixel, unsigned MeshDimension>
unsigned
MeshRepresenter<TPixel, MeshDimension>::FindClosestPoint(const MeshType* mesh, const PointType pt) const {
	throw StatisticalModelException("Not implemented. Currently only points of the reference can be used.");
}

} // namespace itk
