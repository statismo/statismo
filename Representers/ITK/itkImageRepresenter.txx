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


#include "itkImageRepresenter.h"
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


template <class TPixel, unsigned ImageDimension>
ImageRepresenter<TPixel, ImageDimension>::ImageRepresenter()
  : m_reference(0)
{
}
template <class TPixel, unsigned ImageDimension>
ImageRepresenter<TPixel, ImageDimension>::~ImageRepresenter() {
}

template <class TPixel, unsigned ImageDimension>
ImageRepresenter<TPixel, ImageDimension>*
ImageRepresenter<TPixel, ImageDimension>::Clone() const {

	ImageRepresenter* clone = new ImageRepresenter();
	clone->Register();

	typedef itk::ImageDuplicator< DatasetType > DuplicatorType;
	typename DuplicatorType::Pointer duplicator = DuplicatorType::New();
	duplicator->SetInputImage(m_reference);
	duplicator->Update();
	DatasetPointerType clonedReference = duplicator->GetOutput();
	clone->SetReference(clonedReference);
	return clone;
}



template <class TPixel, unsigned ImageDimension>
ImageRepresenter<TPixel, ImageDimension>*
ImageRepresenter<TPixel, ImageDimension>::Load(const H5::CommonFG& fg) {

	ImageRepresenter* newInstance = new ImageRepresenter();
	newInstance->Register();

	std::string tmpfilename = statismo::Utils::CreateTmpName(".vtk");

	HDF5Utils::getFileFromHDF5(fg, "./reference", tmpfilename.c_str());

	newInstance->m_reference = ReadDataset(tmpfilename.c_str());
	std::remove(tmpfilename.c_str());
	return newInstance;
}


template <class TPixel, unsigned ImageDimension>
void
ImageRepresenter<TPixel, ImageDimension>::SetReference(const char* referenceFilename) {

	DatasetPointerType reference = ReadDataset(referenceFilename);
	SetReference(reference);
}

template <class TPixel, unsigned ImageDimension>
void
ImageRepresenter<TPixel, ImageDimension>::SetReference(DatasetPointerType reference) {
	m_reference = reference;
}

template <class TPixel, unsigned ImageDimension>
typename ImageRepresenter<TPixel, ImageDimension>::DatasetPointerType
ImageRepresenter<TPixel, ImageDimension>::DatasetToSample(ImageType* image, DatasetInfo* notUsed) const
{
	// we don't do any alignment for images, but simply return a copy of the image

	typename itk::ImageDuplicator<ImageType>::Pointer duplicator = itk::ImageDuplicator<ImageType>::New();
	duplicator->SetInputImage(image);
	duplicator->Update();
	return duplicator->GetOutput();

}

template <class TPixel, unsigned ImageDimension>
VectorType
ImageRepresenter<TPixel, ImageDimension>::SampleToSampleVector(ImageType* image) const
{
	VectorType sample(GetNumberOfPoints() * GetDimensions());
	itk::ImageRegionConstIterator<DatasetType> it(image, image->GetLargestPossibleRegion());

	it.GoToBegin();
	for (unsigned i = 0;
			it.IsAtEnd() == false;
			++i)
	{
		unsigned idx = MapPointIdToInternalIdx(i, 0);
		sample[idx] = it.Value();
		++it;
	}
	return sample;
}



template <class TPixel, unsigned ImageDimension>
typename ImageRepresenter<TPixel, ImageDimension>::DatasetPointerType
ImageRepresenter<TPixel, ImageDimension>::SampleVectorToSample(const VectorType& sample) const
{

	typedef itk::ImageDuplicator< DatasetType > DuplicatorType;
	typename DuplicatorType::Pointer duplicator = DuplicatorType::New();
	duplicator->SetInputImage(m_reference);
	duplicator->Update();
	DatasetPointerType clonedImage = duplicator->GetOutput();

	itk::ImageRegionIterator<DatasetType> it(clonedImage, clonedImage->GetLargestPossibleRegion());
	it.GoToBegin();
	for (unsigned i  = 0;  !it.IsAtEnd(); ++it, i++) {
		ValueType v;
		unsigned idx = MapPointIdToInternalIdx(i, 0);
		v = sample[idx];
		it.Set(v);
	}
	return clonedImage;
}


template <class TPixel, unsigned ImageDimension>
typename ImageRepresenter<TPixel, ImageDimension>::ValueType
ImageRepresenter<TPixel, ImageDimension>::PointSampleVectorToPointSample(const VectorType& pointSample) const
{
	ValueType value;
	value = pointSample[0];
	return value;
}
template <class TPixel, unsigned ImageDimension>
statismo::VectorType
ImageRepresenter<TPixel, ImageDimension>::PointSampleToPointSampleVector(const ValueType& v) const
{
	VectorType vec(1);
	vec[0] = v;
	return vec;
}


template <class TPixel, unsigned ImageDimension>
void
ImageRepresenter<TPixel, ImageDimension>::Save(const H5::CommonFG& fg) const {
	using namespace H5;

	std::string tmpfilename = statismo::Utils::CreateTmpName(".vtk");

	WriteDataset(tmpfilename.c_str(), (DatasetConstPointerType)this->m_reference);

	HDF5Utils::dumpFileToHDF5(tmpfilename.c_str(), fg, "./reference" );

	std::remove(tmpfilename.c_str());

}


template <class TPixel, unsigned ImageDimension>
unsigned
ImageRepresenter<TPixel, ImageDimension>::GetNumberOfPoints() const {
	return GetNumberOfPoints( (DatasetConstPointerType)this->m_reference);
}


template <class TPixel, unsigned ImageDimension>
unsigned
ImageRepresenter<TPixel, ImageDimension>::GetPointIdForPoint(const PointType& pt) const {
	// itks organization is slice row col
	typename DatasetType::IndexType idx;
	this->m_reference->TransformPhysicalPointToIndex(pt, idx);

	typename DatasetType::SizeType size = this->m_reference->GetLargestPossibleRegion().GetSize();
	// in itk, idx 0 is by convention the fastest moving index
    unsigned int index=0;
    for (unsigned int i=0;i<ImageType::ImageDimension;++i){
        unsigned int multiplier=1;
        for (int d=i-1;d>=0;--d){
            multiplier*=size[d];
        }
        index+=multiplier*idx[i];
    }

    return index;
}


template <class TPixel, unsigned ImageDimension>
typename ImageRepresenter<TPixel, ImageDimension>::DatasetPointerType
ImageRepresenter<TPixel, ImageDimension>::ReadDataset(const char* filename) {
    typename itk::ImageFileReader<ImageType>::Pointer reader = itk::ImageFileReader<ImageType>::New();
    reader->SetFileName(filename);
    try {
        reader->Update();
    }
    catch (itk::ImageFileReaderException& e) {
        throw StatisticalModelException((std::string("Could not read file ") + filename).c_str());
    }

    return reader->GetOutput();
}

template <class TPixel, unsigned ImageDimension>
void ImageRepresenter<TPixel, ImageDimension>::WriteDataset(const char* filename, const ImageType* image) {
    typename itk::ImageFileWriter<ImageType>::Pointer writer = itk::ImageFileWriter<ImageType>::New();
    writer->SetFileName(filename);
    writer->SetInput(image);
    try {
		writer->Update();
    }
    catch (itk::ImageFileWriterException& e) {
        throw StatisticalModelException((std::string("Could not write file ") + filename).c_str());
    }

}


template <class TPixel, unsigned ImageDimension>
unsigned ImageRepresenter<TPixel, ImageDimension>::GetNumberOfPoints(DatasetConstPointerType ds) {
    return ds->GetLargestPossibleRegion().GetNumberOfPixels();
}

} // namespace itk
