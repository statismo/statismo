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


#include "itkVectorImageRepresenterBase.h"
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
#include <iostream>

#ifdef _WIN32
#include <windows.h>
#include <tchar.h>
#endif


namespace itk {

using statismo::VectorType;
using statismo::HDF5Utils;
using statismo::StatisticalModelException;



template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
VectorImageRepresenterBase<TPixel, ImageDimension, VectorDimension>::VectorImageRepresenterBase()
  : m_reference(0)
{
}

template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
VectorImageRepresenterBase<TPixel, ImageDimension, VectorDimension>::~VectorImageRepresenterBase() {
}


template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
void
VectorImageRepresenterBase<TPixel, ImageDimension, VectorDimension>::CloneBaseMembers(VectorImageRepresenterBase* clone) const
{
	typedef itk::ImageDuplicator< DatasetType > DuplicatorType;
	typename DuplicatorType::Pointer duplicator = DuplicatorType::New();
	duplicator->SetInputImage(m_reference);
	duplicator->Update();
	DatasetPointerType clonedReference = duplicator->GetOutput();
	clone->m_reference = clonedReference;
}

template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
void
VectorImageRepresenterBase<TPixel, ImageDimension, VectorDimension>::SetReference(const char* referenceFilename) {

	DatasetPointerType reference = ReadDataset(referenceFilename);
	SetReference(reference);
}

template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
void
VectorImageRepresenterBase<TPixel, ImageDimension, VectorDimension>::SetReference(DatasetPointerType reference) {
	m_reference = reference;
}

template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
VectorType
VectorImageRepresenterBase<TPixel, ImageDimension, VectorDimension>::DatasetToSampleVector(ImageType* image) const
{
	VectorType sample(GetNumberOfPoints() * Dimensions);
	itk::ImageRegionConstIterator<DatasetType> it(image, image->GetLargestPossibleRegion());

	it.GoToBegin();
	for (unsigned i = 0;
			it.IsAtEnd() == false;
			++i)
	{
		for (unsigned j = 0; j < Dimensions; j++) {
			unsigned idx = MapPointIdToInternalIdx(i, j);
			sample[idx] = it.Value()[j];
		}
		++it;
	}
	return sample;
}



template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
typename VectorImageRepresenterBase<TPixel, ImageDimension, VectorDimension>::DatasetPointerType
VectorImageRepresenterBase<TPixel, ImageDimension, VectorDimension>::SampleVectorToSample(const VectorType& sample) const
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
		for (unsigned d = 0; d < Dimensions; d++) {
			unsigned idx = MapPointIdToInternalIdx(i, d);
			v[d] = sample[idx];
		}
		it.Set(v);
	}
	return clonedImage;
}


template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
typename VectorImageRepresenterBase<TPixel, ImageDimension, VectorDimension>::ValueType
VectorImageRepresenterBase<TPixel, ImageDimension, VectorDimension>::PointSampleToValue(const VectorType& pointSample) const
{
	ValueType value;
	for (unsigned i = 0; i < Dimensions; i++) {
		value[i] = pointSample[i];
	}
	return value;
}

template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
statismo::VectorType
VectorImageRepresenterBase<TPixel, ImageDimension, VectorDimension>::ValueToPointSample(const ValueType& v) const
{
	VectorType vec(VectorDimension);
	for (unsigned i = 0; i < vec.size(); i++) {
		vec[i] = v[i];
	}
	return vec;
}

template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
void
VectorImageRepresenterBase<TPixel, ImageDimension, VectorDimension>::LoadBaseMembers(VectorImageRepresenterBase* b, const H5::CommonFG& fg) {

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

	tmpfilename += ".nrrd";
	statismo::HDF5Utils::getFileFromHDF5(fg, "./reference", tmpfilename.c_str());
	b->m_reference = ReadDataset(tmpfilename.c_str());

	std::remove(tmpfilename.c_str());
}

template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
void
VectorImageRepresenterBase<TPixel, ImageDimension, VectorDimension>::Save(const H5::CommonFG& fg) const {
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
	tmpfilename += ".nrrd";
	WriteDataset(tmpfilename.c_str(), (DatasetConstPointerType)this->m_reference);

	statismo::HDF5Utils::dumpFileToHDF5(tmpfilename.c_str(), fg, "./reference" );

	std::remove(tmpfilename.c_str());

}


template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
unsigned
VectorImageRepresenterBase<TPixel, ImageDimension, VectorDimension>::GetNumberOfPoints() const {
	return GetNumberOfPoints( (DatasetConstPointerType)this->m_reference);
}


template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
unsigned
VectorImageRepresenterBase<TPixel, ImageDimension, VectorDimension>::GetPointIdForPoint(const PointType& pt) const {
	// itks organization is slice row col

	typename ImageType::IndexType idx;
	this->m_reference->TransformPhysicalPointToIndex(pt, idx);

	typename DatasetType::SizeType size = this->m_reference->GetLargestPossibleRegion().GetSize();
	// in itk, idx 0 is by convention the fastest moving index
    unsigned int ptId=0;
    for (unsigned int i=0;i<Dimensions;++i){
        unsigned int multiplier=1;
        for (int d=i-1;d>=0;--d){
            multiplier*=size[d];
        }
        ptId+=multiplier*idx[i];
    }
    if (ptId >= GetNumberOfPoints()) {
    	throw statismo::StatisticalModelException("GetPointIdForPoint computed invalid ptId. Make sure that the point is within the reference you chose ");
    }
    return ptId;
}


template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
typename VectorImageRepresenterBase<TPixel, ImageDimension, VectorDimension>::DatasetPointerType
VectorImageRepresenterBase<TPixel, ImageDimension, VectorDimension>::ReadDataset(const char* filename) {
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

template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
void VectorImageRepresenterBase<TPixel, ImageDimension, VectorDimension>::WriteDataset(const char* filename, const ImageType* image) {
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

template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
typename  VectorImageRepresenterBase<TPixel, ImageDimension, VectorDimension>::DatasetPointerType VectorImageRepresenterBase<TPixel, ImageDimension, VectorDimension>::NewDataset() {
    return DatasetType::New();
}


template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
unsigned VectorImageRepresenterBase<TPixel, ImageDimension, VectorDimension>::GetNumberOfPoints(DatasetConstPointerType ds) {
    return ds->GetLargestPossibleRegion().GetNumberOfPixels();
}

} // namespace itk
