/*
 * This file is part of the statismo library.
 *
 * Author: Remi Blanc (rblanc33@gmail.com)
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


#include "itkImageROIRepresenter.h"
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


template <class TPixel, unsigned ImageDimension, class TMaskPixel>
ImageROIRepresenter<TPixel, ImageDimension, TMaskPixel>::ImageROIRepresenter()
  : m_reference(0), m_mask(0)
{
}
template <class TPixel, unsigned ImageDimension, class TMaskPixel>
ImageROIRepresenter<TPixel, ImageDimension, TMaskPixel>::~ImageROIRepresenter() {
}

template <class TPixel, unsigned ImageDimension, class TMaskPixel>
ImageROIRepresenter<TPixel, ImageDimension, TMaskPixel>*
ImageROIRepresenter<TPixel, ImageDimension, TMaskPixel>::Clone() const {

	ImageROIRepresenter* clone = new ImageROIRepresenter();
	clone->Register();

	typedef itk::ImageDuplicator< DatasetType > DuplicatorType;
	typename DuplicatorType::Pointer duplicator = DuplicatorType::New();
	duplicator->SetInputImage(m_reference);
	duplicator->Update();
	DatasetPointerType clonedReference = duplicator->GetOutput();
	
	typedef itk::ImageDuplicator< MaskType > MaskDuplicatorType;
	typename MaskDuplicatorType::Pointer maskDuplicator = MaskDuplicatorType::New();
	maskDuplicator->SetInputImage(m_mask);
	maskDuplicator->Update();
	MaskPointerType clonedMask = maskDuplicator->GetOutput();
	
	clone->SetReference(clonedReference, clonedMask);

	return clone;
}



template <class TPixel, unsigned ImageDimension, class TMaskPixel>
ImageROIRepresenter<TPixel, ImageDimension, TMaskPixel>*
ImageROIRepresenter<TPixel, ImageDimension, TMaskPixel>::Load(const H5::CommonFG& fg) {

	ImageROIRepresenter* newInstance = new ImageROIRepresenter();
	newInstance->Register();

	std::string tmpfilename = statismo::Utils::CreateTmpName(".vtk");
	std::string tmpfilename2 = statismo::Utils::CreateTmpName(".vtk");

	HDF5Utils::getFileFromHDF5(fg, "./reference", tmpfilename.c_str());
	HDF5Utils::getFileFromHDF5(fg, "./referenceMask", tmpfilename2.c_str());

	newInstance->m_reference = ReadDataset(tmpfilename.c_str());
	newInstance->m_mask = ReadMask(tmpfilename2.c_str());

	typename DomainType::DomainPointsListType domainPoints;
  typename MaskType::PixelType* maskBuffer = newInstance->m_mask->GetBufferPointer();

  newInstance->m_mapPointIDToInternalIdx.clear();
  for (unsigned i=0 ; i<newInstance->m_mask->GetLargestPossibleRegion().GetNumberOfPixels () ; i++) {
    if ( maskBuffer[i] != 0 ) {
      newInstance->m_mapPointIDToInternalIdx.push_back( domainPoints.size() ); //-> has as many entries as the number of pixels
      domainPoints.push_back(i); //-> to the index in the domain corresponds the pixel index (id) in the image
    } else newInstance->m_mapPointIDToInternalIdx.push_back( std::numeric_limits<unsigned>::max() );
  }

  newInstance->m_domain = DomainType(domainPoints);


  std::remove(tmpfilename.c_str());
  std::remove(tmpfilename2.c_str());
	return newInstance;
}



template <class TPixel, unsigned ImageDimension, class TMaskPixel>
void
ImageROIRepresenter<TPixel, ImageDimension, TMaskPixel>::SetReference(ImageType* reference, MaskType* mask) {

  if ( !mask ) { //default is the whole image
    m_mask = MaskType::New();
    m_mask->SetRegions(reference->GetLargestPossibleRegion());
    m_mask->SetSpacing(reference->GetSpacing());
    m_mask->SetOrigin(reference->GetOrigin());
    m_mask->Allocate();
    m_mask->FillBuffer(1);
  }
  else { //check the mask & reference image are consistent.
    for (unsigned i=0 ; i<ImageDimension ; i++) {
      if ( mask->GetLargestPossibleRegion().GetSize(i) != reference->GetLargestPossibleRegion().GetSize(i) ) throw StatisticalModelException("the reference image and mask must have the same size, spacing and origin");
    }
		m_mask = mask;
  }
  m_reference = reference;
	
	typename DomainType::DomainPointsListType domainPoints;
  typename MaskType::PixelType* maskBuffer = m_mask->GetBufferPointer();

  m_mapPointIDToInternalIdx.clear();
  for (unsigned i=0 ; i<m_mask->GetLargestPossibleRegion().GetNumberOfPixels () ; i++) {
    if ( maskBuffer[i] != 0 ) {
      m_mapPointIDToInternalIdx.push_back( domainPoints.size() ); //-> has as many entries as the number of pixels
      domainPoints.push_back(i); //-> to the index in the domain corresponds the pixel index (id) in the image
    } else m_mapPointIDToInternalIdx.push_back( std::numeric_limits<unsigned>::max() );
  }

  m_domain = DomainType(domainPoints);
}

template <class TPixel, unsigned ImageDimension, class TMaskPixel>
typename ImageROIRepresenter<TPixel, ImageDimension, TMaskPixel>::DatasetPointerType
ImageROIRepresenter<TPixel, ImageDimension, TMaskPixel>::DatasetToSample(ImageType* image, DatasetInfo* notUsed) const
{
	// we don't do any alignment for images, but simply return a copy of the image
//TODO: remove the background pixels?
//TODO: avoid duplicating anything in the standard itkImageRepresenter - save time & memory ??
	typename itk::ImageDuplicator<ImageType>::Pointer duplicator = itk::ImageDuplicator<ImageType>::New();
	duplicator->SetInputImage(image);
	duplicator->Update();
	return duplicator->GetOutput();

}

template <class TPixel, unsigned ImageDimension, class TMaskPixel>
VectorType
ImageROIRepresenter<TPixel, ImageDimension, TMaskPixel>::SampleToSampleVector(ImageType* image) const
{

    //check the dimensions of the input image...
    for (unsigned i=0 ; i<ImageDimension ; i++) {
        if ( image->GetLargestPossibleRegion().GetSize(i) != m_reference->GetLargestPossibleRegion().GetSize(i) )
            throw StatisticalModelException("the input image must be defined on the same grid as the reference");
    }
    
	VectorType sample(GetNumberOfPoints() * GetDimensions());
	
	ValueType* buffer = image->GetBufferPointer();
	
	typename DomainType::DomainPointsListType::const_iterator it;
	unsigned i;
	for ( it = m_domain.GetDomainPoints().begin(), i=0 ; it != m_domain.GetDomainPoints().end() ; ++it, ++i) {
	    sample[i] = buffer[ *it ];
	}
	
	return sample;
}



template <class TPixel, unsigned ImageDimension, class TMaskPixel>
typename ImageROIRepresenter<TPixel, ImageDimension, TMaskPixel>::DatasetPointerType
ImageROIRepresenter<TPixel, ImageDimension, TMaskPixel>::SampleVectorToSample(const VectorType& sample) const
{

	typedef itk::ImageDuplicator< DatasetType > DuplicatorType;
	typename DuplicatorType::Pointer duplicator = DuplicatorType::New();
	duplicator->SetInputImage(m_reference);
	duplicator->Update();
	DatasetPointerType clonedImage = duplicator->GetOutput();

	ValueType* buffer = clonedImage->GetBufferPointer();
    
	typename DomainType::DomainPointsListType::const_iterator it;
	unsigned idx;
	for ( it = m_domain.GetDomainPoints().begin(), idx=0 ; it != m_domain.GetDomainPoints().end() ; ++it, ++idx) {
	    buffer[ *it ] = sample[idx];
	}
    
	return clonedImage;
}

template <class TPixel, unsigned ImageDimension, class TMaskPixel>
typename ImageROIRepresenter<TPixel, ImageDimension, TMaskPixel>::ValueType
ImageROIRepresenter<TPixel, ImageDimension, TMaskPixel>::PointSampleFromSample(DatasetConstPointerType sample, unsigned ptid) const
{
	if (ptid >= GetDomain().GetNumberOfPoints()) {
		throw StatisticalModelException("invalid ptid provided to PointSampleFromSample");
	}

    ValueType *buffer = sample->GetBufferPointer();
    return buffer[ m_domain.GetDomainPoints()[ptid] ];
    
}

template <class TPixel, unsigned ImageDimension, class TMaskPixel>
typename ImageROIRepresenter<TPixel, ImageDimension, TMaskPixel>::ValueType
ImageROIRepresenter<TPixel, ImageDimension, TMaskPixel>::PointSampleVectorToPointSample(const VectorType& pointSample) const
{
	ValueType value;
	value = pointSample[0];
	return value;
}

template <class TPixel, unsigned ImageDimension, class TMaskPixel>
statismo::VectorType
ImageROIRepresenter<TPixel, ImageDimension, TMaskPixel>::PointSampleToPointSampleVector(const ValueType& v) const
{
	VectorType vec(1);
	vec[0] = v;
	return vec;
}


template <class TPixel, unsigned ImageDimension, class TMaskPixel>
void
ImageROIRepresenter<TPixel, ImageDimension, TMaskPixel>::Save(const H5::CommonFG& fg) const {
	using namespace H5;

	std::string tmpfilename = statismo::Utils::CreateTmpName(".vtk");
	std::string tmpfilename2 = statismo::Utils::CreateTmpName(".vtk");

	WriteDataset(tmpfilename.c_str(), (DatasetConstPointerType)this->m_reference);
	WriteMask(tmpfilename2.c_str(), (MaskConstPointerType)this->m_mask);

	HDF5Utils::dumpFileToHDF5(tmpfilename.c_str(), fg, "./reference" );
	HDF5Utils::dumpFileToHDF5(tmpfilename2.c_str(), fg, "./referenceMask" );

	std::remove(tmpfilename.c_str());
	std::remove(tmpfilename2.c_str());

}


template <class TPixel, unsigned ImageDimension, class TMaskPixel>
unsigned
ImageROIRepresenter<TPixel, ImageDimension, TMaskPixel>::GetNumberOfPoints() const {
	return m_domain.GetNumberOfPoints();
}


template <class TPixel, unsigned ImageDimension, class TMaskPixel>
unsigned
ImageROIRepresenter<TPixel, ImageDimension, TMaskPixel>::GetPointIdForPoint(const PointType& pt) const {
	//// itks organization is slice row col
	//typename DatasetType::IndexType idx;
	//this->m_reference->TransformPhysicalPointToIndex(pt, idx);

	//typename DatasetType::SizeType size = this->m_reference->GetLargestPossibleRegion().GetSize();
	//// in itk, idx 0 is by convention the fastest moving index
    //unsigned int index=0;
    //for (unsigned int i=0;i<ImageType::ImageDimension;++i){
    //    unsigned int multiplier=1;
    //    for (int d=i-1;d>=0;--d){
    //        multiplier*=size[d];
    //    }
    //    index+=multiplier*idx[i];
    //}

    //return index;
    return pt; //in this case, I use directly the index as ID...
}


template <class TPixel, unsigned ImageDimension, class TMaskPixel>
typename ImageROIRepresenter<TPixel, ImageDimension, TMaskPixel>::DatasetPointerType
ImageROIRepresenter<TPixel, ImageDimension, TMaskPixel>::ReadDataset(const char* filename) {
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

template <class TPixel, unsigned ImageDimension, class TMaskPixel>
typename ImageROIRepresenter<TPixel, ImageDimension, TMaskPixel>::MaskPointerType
ImageROIRepresenter<TPixel, ImageDimension, TMaskPixel>::ReadMask(const char* filename) {
    typename itk::ImageFileReader<MaskType>::Pointer reader = itk::ImageFileReader<MaskType>::New();
    reader->SetFileName(filename);
    try {
        reader->Update();
    }
    catch (itk::ImageFileReaderException& e) {
        throw StatisticalModelException((std::string("Could not read file ") + filename).c_str());
    }

    return reader->GetOutput();
}


template <class TPixel, unsigned ImageDimension, class TMaskPixel>
void ImageROIRepresenter<TPixel, ImageDimension, TMaskPixel>::WriteDataset(const char* filename, const ImageType* image) {
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

template <class TPixel, unsigned ImageDimension, class TMaskPixel>
void ImageROIRepresenter<TPixel, ImageDimension, TMaskPixel>::WriteMask(const char* filename, const MaskType* image) {
    typename itk::ImageFileWriter<MaskType>::Pointer writer = itk::ImageFileWriter<MaskType>::New();
    writer->SetFileName(filename);
    writer->SetInput(image);
    try {
		writer->Update();
    }
    catch (itk::ImageFileWriterException& e) {
        throw StatisticalModelException((std::string("Could not write file ") + filename).c_str());
    }

}


template <class TPixel, unsigned ImageDimension, class TMaskPixel>
unsigned ImageROIRepresenter<TPixel, ImageDimension, TMaskPixel>::GetNumberOfPoints(DatasetConstPointerType ds) {
    return m_domain.GetNumberOfPoints();
}

} // namespace itk
