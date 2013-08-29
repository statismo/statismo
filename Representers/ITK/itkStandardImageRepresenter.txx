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
#include "itkStandardImageRepresenter.h"


namespace itk {

using statismo::VectorType;
using statismo::HDF5Utils;
using statismo::StatisticalModelException;


template <class TPixel, unsigned ImageDimension>
StandardImageRepresenter<TPixel, ImageDimension>::StandardImageRepresenter()
  : m_reference(0)
{
}
template <class TPixel, unsigned ImageDimension>
StandardImageRepresenter<TPixel, ImageDimension>::~StandardImageRepresenter() {
}

template <class TPixel, unsigned ImageDimension>
StandardImageRepresenter<TPixel, ImageDimension>*
StandardImageRepresenter<TPixel, ImageDimension>::Clone() const {

	StandardImageRepresenter* clone = new StandardImageRepresenter();
	clone->Register();

	DatasetPointerType clonedReference = this->CloneDataset(m_reference);
	clone->SetReference(clonedReference);
	return clone;
}



template <class TPixel, unsigned ImageDimension>
void
StandardImageRepresenter<TPixel, ImageDimension>::Load(const H5::Group& fg) {



	int readImageDimension = HDF5Utils::readInt(fg, "imageDimension");
	if (readImageDimension != ImageDimension)  {
		throw statismo::StatisticalModelException("the image dimension specified in the statismo file does not match the one specified as template parameter");
	}


	statismo::VectorType originVec;
	HDF5Utils::readVector(fg, "origin", originVec);
	typename ImageType::PointType origin;
	for (unsigned i = 0; i < ImageDimension; i++) {
		origin[i] = originVec[i];
	}

	statismo::VectorType spacingVec;
	HDF5Utils::readVector(fg, "spacing", spacingVec);
	typename ImageType::SpacingType spacing;
	for (unsigned i = 0; i < ImageDimension; i++) {
		spacing[i] = spacingVec[i];
	}

	typename statismo::GenericEigenType<int>::VectorType sizeVec;
	HDF5Utils::readVectorOfType<int>(fg, "size", sizeVec);
	typename ImageType::SizeType size;
	for (unsigned i = 0; i < ImageDimension; i++) {
		size[i] = sizeVec[i];
	}

	statismo::MatrixType directionMat;
	HDF5Utils::readMatrix(fg, "direction", directionMat);
	typename ImageType::DirectionType direction;
	for (unsigned i = 0; i < directionMat.rows(); i++) {
		for (unsigned j = 0; j < directionMat.rows(); j++) {
			direction[i][j] = directionMat(i,j);
		}
	}

	H5::Group pdGroup = fg.openGroup("./pointData");
	int readPixelDimension = HDF5Utils::readInt(pdGroup, "pixelDimension");
	if (readPixelDimension != GetDimensions())  {
		throw statismo::StatisticalModelException("the pixel dimension specified in the statismo file does not match the one specified as template parameter");
	}

	typename statismo::GenericEigenType<double>::MatrixType pixelMat;
	HDF5Utils::readMatrixOfType<double>(pdGroup, "pixelValues", pixelMat);
	typename ImageType::Pointer newImage = ImageType::New();
	typename ImageType::IndexType start;
	start.Fill(0);


	H5::DataSet ds = pdGroup.openDataSet("pixelValues");
	int type = HDF5Utils::readIntAttribute(ds, "datatype");
	if (type != PixelConversionTrait<TPixel>::GetDataType()) {
		std::cout << "Warning: The datatype specified for the scalars does not match the TPixel template argument used in this representer." << std::endl;
	}
	pdGroup.close();
	typename ImageType::RegionType region(start, size);
	newImage->SetRegions(region);
	newImage->Allocate();
	newImage->SetOrigin(origin);
	newImage->SetSpacing(spacing);
	newImage->SetDirection(direction);


	itk::ImageRegionIterator<DatasetType> it(newImage, newImage->GetLargestPossibleRegion());
	it.GoToBegin();
	for (unsigned i  = 0;  !it.IsAtEnd(); ++it, i++) {
		TPixel v = PixelConversionTrait<TPixel>::FromVector(pixelMat.col(i));
		it.Set(v);
	}

	this->SetReference(newImage);
}



template <class TPixel, unsigned ImageDimension>
void
StandardImageRepresenter<TPixel, ImageDimension>::SetReference(ImageType* reference) {
	m_reference = reference;

	typename DomainType::DomainPointsListType domainPoints;
	itk::ImageRegionConstIterator<DatasetType> it(reference, reference->GetLargestPossibleRegion());
	it.GoToBegin();
	for (;
		it.IsAtEnd() == false
		;)
	{
		PointType pt;
		reference->TransformIndexToPhysicalPoint(it.GetIndex(), pt);
		domainPoints.push_back(pt);
		++it;
	}
	m_domain = DomainType(domainPoints);
}

template <class TPixel, unsigned ImageDimension>
typename StandardImageRepresenter<TPixel, ImageDimension>::DatasetPointerType
StandardImageRepresenter<TPixel, ImageDimension>::DatasetToSample(DatasetConstPointerType image) const
{
	// we don't do any alignment for images, but simply return a copy of the image

	typename itk::ImageDuplicator<ImageType>::Pointer duplicator = itk::ImageDuplicator<ImageType>::New();
	duplicator->SetInputImage(image);
	duplicator->Update();
	return duplicator->GetOutput();

}

template <class TPixel, unsigned ImageDimension>
VectorType
StandardImageRepresenter<TPixel, ImageDimension>::SampleToSampleVector(DatasetConstPointerType image) const
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


template <class TPixel, unsigned ImageDimension>
typename StandardImageRepresenter<TPixel, ImageDimension>::DatasetPointerType
StandardImageRepresenter<TPixel, ImageDimension>::SampleVectorToSample(const VectorType& sample) const
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

template <class TPixel, unsigned ImageDimension>
typename StandardImageRepresenter<TPixel, ImageDimension>::ValueType
StandardImageRepresenter<TPixel, ImageDimension>::PointSampleFromSample(DatasetConstPointerType sample, unsigned ptid) const
{
	if (ptid >= GetDomain().GetNumberOfPoints()) {
		throw StatisticalModelException("invalid ptid provided to PointSampleFromSample");
	}

	// we get the point with the id from the domain, as itk does not allow us get a point via its index.
	PointType pt = GetDomain().GetDomainPoints()[ptid];
	typename ImageType::IndexType idx;
	sample->TransformPhysicalPointToIndex(pt, idx);
	ValueType value = sample->GetPixel(idx);
	return value;

}

template <class TPixel, unsigned ImageDimension>
typename StandardImageRepresenter<TPixel, ImageDimension>::ValueType
StandardImageRepresenter<TPixel, ImageDimension>::PointSampleVectorToPointSample(const VectorType& pointSample) const
{
	ValueType value;
	for (unsigned i = 0; i < GetDimensions(); i++) {
		value[i] = pointSample[i];
	}
	return value;
}

template <class TPixel, unsigned ImageDimension>
statismo::VectorType
StandardImageRepresenter<TPixel, ImageDimension>::PointSampleToPointSampleVector(const ValueType& v) const
{
	VectorType vec(GetDimensions());
	for (unsigned i = 0; i < vec.size(); i++) {
		vec[i] = v[i];
	}
	return vec;
}


template <class TPixel, unsigned ImageDimension>
void
StandardImageRepresenter<TPixel, ImageDimension>::Save(const H5::Group& fg) const {
	using namespace H5;


	typename ImageType::PointType origin = m_reference->GetOrigin();
	statismo::VectorType originVec(ImageDimension);
	for (unsigned i = 0; i < ImageDimension; i++) {
		originVec(i) = origin[i];
	}
	HDF5Utils::writeVector(fg, "origin", originVec);

	typename ImageType::SpacingType spacing = m_reference->GetSpacing();
	statismo::VectorType spacingVec(ImageDimension);
	for (unsigned i = 0; i < ImageDimension; i++) {
		spacingVec(i) = spacing[i];
	}
	HDF5Utils::writeVector(fg, "spacing", spacingVec);


	statismo::GenericEigenType<int>::VectorType sizeVec(ImageDimension);
	for (unsigned i = 0; i < ImageDimension; i++) {
		sizeVec(i) = m_reference->GetLargestPossibleRegion().GetSize()[i];
	}
	HDF5Utils::writeVectorOfType<int>(fg, "size", sizeVec);

	typename ImageType::DirectionType direction = m_reference->GetDirection();
	statismo::MatrixType directionMat(ImageDimension, ImageDimension);
	for (unsigned i = 0; i < ImageDimension; i++) {
		for (unsigned j = 0; j < ImageDimension; j++) {
			directionMat(i,j) = direction[i][j];
		}
	}
	HDF5Utils::writeMatrix(fg, "direction", directionMat);

	HDF5Utils::writeInt(fg, "imageDimension", ImageDimension);

	H5::Group pdGroup = fg.createGroup("pointData");
	HDF5Utils::writeInt(pdGroup, "pixelDimension", GetDimensions());


	typedef statismo::GenericEigenType<double>::MatrixType DoubleMatrixType;
	DoubleMatrixType pixelMat(GetDimensions(), GetNumberOfPoints());

	itk::ImageRegionIterator<DatasetType> it(m_reference, m_reference->GetLargestPossibleRegion());
	it.GoToBegin();
	for (unsigned i = 0;
			it.IsAtEnd() == false;
			++i)
	{
		pixelMat.col(i) = PixelConversionTrait<TPixel>::ToVector(it.Get());
		++it;
	}
	H5::DataSet ds = HDF5Utils::writeMatrixOfType<double>(pdGroup, "pixelValues", pixelMat);
	HDF5Utils::writeIntAttribute(ds, "datatype", PixelConversionTrait<TPixel>::GetDataType());
	pdGroup.close();
}


template <class TPixel, unsigned ImageDimension>
unsigned
StandardImageRepresenter<TPixel, ImageDimension>::GetNumberOfPoints() const {
	return m_reference->GetLargestPossibleRegion().GetNumberOfPixels();
}


template <class TPixel, unsigned ImageDimension>
unsigned
StandardImageRepresenter<TPixel, ImageDimension>::GetPointIdForPoint(const PointType& pt) const {
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
typename StandardImageRepresenter<TPixel, ImageDimension>::DatasetPointerType
StandardImageRepresenter<TPixel, ImageDimension>::CloneDataset(DatasetConstPointerType d) const {
	typedef itk::ImageDuplicator< DatasetType > DuplicatorType;
	typename DuplicatorType::Pointer duplicator = DuplicatorType::New();
	duplicator->SetInputImage(d);
	duplicator->Update();
	DatasetPointerType clonedReference = duplicator->GetOutput();
	return clonedReference;
}

} // namespace itk
