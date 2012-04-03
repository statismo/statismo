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


#include "itkVectorImageLMAlignRepresenter.h"
#include "itkImageIterator.h"
#include "itkImageRegionConstIterator.h"
#include "itkImageRegionIterator.h"
#include "itkImageDuplicator.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkIndex.h"
#include "itkPoint.h"
#include "itkVector.h"
#include "itkLandmarkBasedTransformInitializer.h"
#include "itkVectorResampleImageFilter.h"

#include "statismo/HDF5Utils.h"
#include <iostream>

#ifdef _WIN32
#include <windows.h>
#include <tchar.h>
#endif


namespace itk {

using statismo::VectorType;
using statismo::HDF5Utils;
using statismo::StatisticalModelException;

template <unsigned Dimensions>
struct TransformSelector {
};
template <>
struct TransformSelector<2> {
	typedef itk::Rigid2DTransform<double> TransformType;
};
template <>
struct TransformSelector<3> {
	typedef itk::VersorRigid3DTransform<double> TransformType;
};


template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
VectorImageLMAlignRepresenter<TPixel, ImageDimension, VectorDimension>::VectorImageLMAlignRepresenter()
{
}

template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
VectorImageLMAlignRepresenter<TPixel, ImageDimension, VectorDimension>::~VectorImageLMAlignRepresenter() {
}



template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
VectorImageLMAlignRepresenter<TPixel, ImageDimension, VectorDimension>*
VectorImageLMAlignRepresenter<TPixel, ImageDimension, VectorDimension>::Clone() const {
	VectorImageLMAlignRepresenter* clone = new VectorImageLMAlignRepresenter();
	clone->Register();
	CloneBaseMembers(clone);
	clone->m_alignmentList = m_alignmentList;
	return clone;
}

template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
VectorImageLMAlignRepresenter<TPixel, ImageDimension, VectorDimension>*
VectorImageLMAlignRepresenter<TPixel, ImageDimension, VectorDimension>::Load(const H5::CommonFG& fg) {
	VectorImageLMAlignRepresenter* newInstance = new VectorImageLMAlignRepresenter();
	newInstance->Register();

	Superclass::LoadBaseMembers(newInstance, fg);
	unsigned num_landmarks = statismo::HDF5Utils::readInt(fg, "numberOfLandmarks");

	for (unsigned i = 0; i < num_landmarks; i++) {
		std::ostringstream ss;
		ss << "landmark_" << i;

		statismo::VectorType v(ImageDimension);
		statismo::HDF5Utils::readVector(fg, ss.str().c_str(), v);
		PointType pt;
		for (unsigned i = 0; i < ImageDimension; i++) {
			pt[i] = v[i];
		}
		newInstance->m_alignmentList.push_back(pt);
	}
	return newInstance;
}


template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
void
VectorImageLMAlignRepresenter<TPixel, ImageDimension, VectorDimension>::SetAlignmentPoints(const AlignmentListType& alignmentList) {
	m_alignmentList = alignmentList;
}

template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
typename VectorImageLMAlignRepresenter<TPixel, ImageDimension, VectorDimension>::DatasetPointerType
VectorImageLMAlignRepresenter<TPixel, ImageDimension, VectorDimension>::DatasetToSample(DatasetType* vecImage, DatasetInfo* notUsed) const
{

	typedef typename TransformSelector<ImageDimension>::TransformType TransformType;
	typedef LandmarkBasedTransformInitializer<TransformType, DatasetType, DatasetType > TransformInitializerType;
	typename TransformType::Pointer transform = TransformType::New();
	typename TransformInitializerType::Pointer transformInitializer = TransformInitializerType::New();

	typedef VectorResampleImageFilter<DatasetType, DatasetType, double> VectorResampleImageFilterType;

	AlignmentListType targetLandmarks;
	for (typename AlignmentListType::const_iterator it = m_alignmentList.begin();
		it != m_alignmentList.end();
		it++) {
			PointType pt = *it;
			PointType targetPt;

			typename DatasetType::IndexType idx;
			vecImage->TransformPhysicalPointToIndex(pt, idx);
			typename DatasetType::PixelType displacement = vecImage->GetPixel(idx);

			for (unsigned i =0; i < VectorDimension; i++) {
				targetPt[i] = pt[i] + displacement[i];
			}
			targetLandmarks.push_back(targetPt);
		}

	transformInitializer->SetTransform(transform);
	transformInitializer->SetFixedLandmarks(m_alignmentList);
	transformInitializer->SetMovingLandmarks(targetLandmarks);
	transformInitializer->InitializeTransform();


	typename VectorResampleImageFilterType::Pointer resampler =  VectorResampleImageFilterType::New();
	resampler->SetTransform(transform);
	resampler->SetInput(vecImage);
	resampler->SetOutputSpacing(this->m_reference->GetSpacing());
	resampler->SetSize(this->m_reference->GetLargestPossibleRegion().GetSize());
	resampler->SetOutputOrigin(this->m_reference->GetOrigin());

	resampler->Update();

	DatasetPointerType resampledDf = resampler->GetOutput();

	return resampledDf;
}


template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
VectorType
VectorImageLMAlignRepresenter<TPixel, ImageDimension, VectorDimension>::SampleToSampleVector(DatasetType* sampleDf) const
{


	// now convert the resampled df
	VectorType sample = VectorType::Zero(this->GetNumberOfPoints() * GetDimensions());
	itk::ImageRegionConstIterator<DatasetType> it(sampleDf, sampleDf->GetLargestPossibleRegion());

	it.GoToBegin();
	for (unsigned i = 0;
			it.IsAtEnd() == false;
			++i)
	{
		for (unsigned j = 0; j < GetDimensions(); j++) {
			unsigned idx = this->MapPointIdToInternalIdx(i, j);
			sample(idx) = it.Value()[j];
		}
		++it;
	}
	return sample;
}



template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
typename VectorImageLMAlignRepresenter<TPixel, ImageDimension, VectorDimension>::DatasetPointerType
VectorImageLMAlignRepresenter<TPixel, ImageDimension, VectorDimension>::SampleVectorToSample(const VectorType& sample) const
{

	typedef itk::ImageDuplicator< DatasetType > DuplicatorType;
	typename DuplicatorType::Pointer duplicator = DuplicatorType::New();
	duplicator->SetInputImage(this->m_reference);
	duplicator->Update();
	DatasetPointerType clonedImage = duplicator->GetOutput();

	itk::ImageRegionIterator<DatasetType> it(clonedImage, clonedImage->GetLargestPossibleRegion());
	it.GoToBegin();
	for (unsigned i  = 0;  !it.IsAtEnd(); ++it, i++) {
		ValueType v;
		for (unsigned d = 0; d < GetDimensions(); d++) {
			unsigned idx = this->MapPointIdToInternalIdx(i, d);
			v[d] = sample[idx];
		}
		it.Set(v);
	}
	return clonedImage;
}


template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
typename VectorImageLMAlignRepresenter<TPixel, ImageDimension, VectorDimension>::ValueType
VectorImageLMAlignRepresenter<TPixel, ImageDimension, VectorDimension>::PointSampleVectorToPointSample(const VectorType& pointSample) const
{
	ValueType value;
	for (unsigned i = 0; i < GetDimensions(); i++) {
		value[i] = pointSample[i];
	}
	return value;
}

template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
statismo::VectorType
VectorImageLMAlignRepresenter<TPixel, ImageDimension, VectorDimension>::PointSampleToPointSampleVector(const ValueType& v) const
{
	VectorType vec(VectorDimension);
	for (unsigned i = 0; i < vec.size(); i++) {
		vec[i] = v[i];
	}
	return vec;
}



template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
void
VectorImageLMAlignRepresenter<TPixel, ImageDimension, VectorDimension>::Save(const H5::CommonFG& fg) const {
	Superclass::Save(fg);
	statismo::HDF5Utils::writeInt(fg, "numberOfLandmarks", m_alignmentList.size());
	for (typename AlignmentListType::const_iterator it = m_alignmentList.begin();
		it != m_alignmentList.end();
		it++)
	{
		statismo::VectorType v(ImageDimension);
		for (unsigned d = 0; d < ImageDimension; d++) {
			v[d] = (*it)[d];
		}
		std::ostringstream ss;
		ss << "landmark_" << it - m_alignmentList.begin();
		statismo::HDF5Utils::writeVector(fg, ss.str().c_str(), v);
	}
}



} // namespace itk
