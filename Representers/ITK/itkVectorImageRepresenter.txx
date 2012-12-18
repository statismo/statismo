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


#include "itkVectorImageRepresenter.h"
#include "itkImageIterator.h"
#include "itkImageRegionConstIterator.h"
#include "itkImageRegionIterator.h"
#include "itkImageDuplicator.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkIndex.h"
#include "itkPoint.h"
#include "itkVector.h"
#include "statismo/HDF5Utils.h"
#include "statismo/utils.h"
#include <iostream>


namespace itk {

using statismo::VectorType;
using statismo::HDF5Utils;
using statismo::StatisticalModelException;



template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
VectorImageRepresenter<TPixel, ImageDimension, VectorDimension>::VectorImageRepresenter()
{
}

template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
VectorImageRepresenter<TPixel, ImageDimension, VectorDimension>::~VectorImageRepresenter() {
}



template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
VectorImageRepresenter<TPixel, ImageDimension, VectorDimension>*
VectorImageRepresenter<TPixel, ImageDimension, VectorDimension>::Clone() const {
	VectorImageRepresenter* clone = new VectorImageRepresenter();
	clone->Register();
	this->CloneBaseMembers(clone);
	return clone;
}

template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
VectorImageRepresenter<TPixel, ImageDimension, VectorDimension>*
VectorImageRepresenter<TPixel, ImageDimension, VectorDimension>::Load(const H5::CommonFG& fg) {
	VectorImageRepresenter* newInstance = new VectorImageRepresenter();
	newInstance->Register();

	Superclass::LoadBaseMembers(newInstance, fg);
	return newInstance;
}

template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
typename VectorImageRepresenter<TPixel, ImageDimension, VectorDimension>::DatasetPointerType
VectorImageRepresenter<TPixel, ImageDimension, VectorDimension>::DatasetToSample(DatasetType* image, DatasetInfo* notUsed) const
{
	// we don't do any alignment for images, but simply return a copy of the image
	typename itk::ImageDuplicator<DatasetType>::Pointer duplicator = itk::ImageDuplicator<DatasetType>::New();
	duplicator->SetInputImage(image);
	duplicator->Update();
	return duplicator->GetOutput();

}

template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
VectorType
VectorImageRepresenter<TPixel, ImageDimension, VectorDimension>::SampleToSampleVector(DatasetType* image) const
{
	VectorType sample(this->GetNumberOfPoints() * GetDimensions());
	itk::ImageRegionConstIterator<DatasetType> it(image, image->GetLargestPossibleRegion());

	it.GoToBegin();
	for (unsigned i = 0;
			it.IsAtEnd() == false;
			++i)
	{
		for (unsigned j = 0; j < GetDimensions(); j++) {
			unsigned idx = this->MapPointIdToInternalIdx(i, j);
			sample[idx] = it.Value()[j];
		}
		++it;
	}
	return sample;
}



template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
typename VectorImageRepresenter<TPixel, ImageDimension, VectorDimension>::DatasetPointerType
VectorImageRepresenter<TPixel, ImageDimension, VectorDimension>::SampleVectorToSample(const VectorType& sample) const
{
	if (this->m_reference.GetPointer() == 0) {
		itkExceptionMacro(<< "Reference must be set before the representer can be used");
	}
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
typename VectorImageRepresenter<TPixel, ImageDimension, VectorDimension>::ValueType
VectorImageRepresenter<TPixel, ImageDimension, VectorDimension>::PointSampleVectorToPointSample(const VectorType& pointSample) const
{
	ValueType value;
	for (unsigned i = 0; i < GetDimensions(); i++) {
		value[i] = pointSample[i];
	}
	return value;
}

template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
statismo::VectorType
VectorImageRepresenter<TPixel, ImageDimension, VectorDimension>::PointSampleToPointSampleVector(const ValueType& v) const
{
	VectorType vec(VectorDimension);
	for (unsigned i = 0; i < vec.size(); i++) {
		vec[i] = v[i];
	}
	return vec;
}

} // namespace itk
