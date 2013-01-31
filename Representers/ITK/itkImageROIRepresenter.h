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



#ifndef ITK_IMAGE_ROI_REPRESENTER_H_
#define ITK_IMAGE_ROI_REPRESENTER_H_

#include "itkImage.h"
#include "statismo_ITK/statismoITKConfig.h"
#include "statismo/CommonTypes.h"
#include "itkObject.h"
#include <H5Cpp.h>

namespace itk {

/**
 * \ingroup Representers
 * \brief A representer for scalar valued itk Images, restricted within a ROI specified through a binary mask
 * \warning the images must be defined on the exact same grid of pixels (same number of pixels in every dimension)
 * \sa Representer
 */

template <class TPixel, unsigned ImageDimension, class TMaskPixel = unsigned char>
class ImageROIRepresenter : public Object {
public:

	/* Standard class typedefs. */
	typedef ImageROIRepresenter       Self;
	typedef Object	                  Superclass;
	typedef SmartPointer<Self>        Pointer;
	typedef SmartPointer<const Self>  ConstPointer;


	/** New macro for creation of through a Smart Pointer. */
	itkSimpleNewMacro( Self );

	/** Run-time type information (and related methods). */
	itkTypeMacro( ImageROIRepresenter, Object );

	static ImageROIRepresenter* Load(const H5::CommonFG& fg);
	ImageROIRepresenter* Clone() const;

    typedef itk::Image<TPixel, ImageDimension>     ImageType;
    typedef itk::Image<TMaskPixel, ImageDimension> MaskType;

	/// The type of the data set to be used
	typedef ImageType DatasetType;

	// Const correcness is difficult to enforce using smart pointers, as no conversion
	// between nonconst and const pointer are possible. Thus we define both to be non-const
	typedef typename ImageType::Pointer DatasetPointerType;
	typedef typename ImageType::Pointer DatasetConstPointerType;
	typedef typename MaskType::Pointer  MaskPointerType;
	typedef typename MaskType::Pointer  MaskConstPointerType;


	typedef unsigned PointType;
	typedef typename ImageType::PixelType ValueType;

	typedef typename MaskType::PointType MaskPointType;

	typedef statismo::Domain<PointType> DomainType;

	struct DatasetInfo {}; // not used for this representer, but needs to be here as it is part of the generic interface

	ImageROIRepresenter();
	virtual ~ImageROIRepresenter();

	static unsigned GetDimensions() { return 1; }
	static std::string GetName() { return "itkImageROIRepresenter"; }

	const DomainType& GetDomain() const { return m_domain; }


	/** Set the reference that is used to build the model
	 * if the mask is not specified, the full image is used,
	 * otherwise the pixels with mask value 0 are excluded from the model
	 * any non-zero value is considered 'inside'
	 */
	void SetReference(ImageType* ds, MaskType* mask = NULL);


	/**
	 * Creates a sample by first aligning the dataset ds to the reference using Procrustes
	 * Alignment.
	 */
	DatasetPointerType DatasetToSample(ImageType* ds, DatasetInfo* notUsed) const;
	statismo::VectorType SampleToSampleVector(ImageType* sample) const;
	DatasetPointerType SampleVectorToSample(const statismo::VectorType& sample) const;

	ValueType PointSampleFromSample(DatasetConstPointerType sample, unsigned ptid) const;
	ValueType PointSampleVectorToPointSample(const statismo::VectorType& pointSample) const;
	statismo::VectorType PointSampleToPointSampleVector(const ValueType& v) const;

	void Save(const H5::CommonFG& fg) const;
	virtual unsigned GetNumberOfPoints() const;
	virtual unsigned GetPointIdForPoint(const PointType& point) const;

	 /* Maps a (Pointid,component) tuple to a component of the internal matrix.
	 * This is used to locate the place in the matrix to store the elements for a given point.
	 * @params ptId The point id
	 * @params the Component Index (range 0, Dimensionality)
	 * @returns an index.
	 */
	unsigned MapPointIdToInternalIdx(unsigned ptId, unsigned componentInd) {
		return m_mapPointIDToInternalIdx[ptId];
	}

    unsigned GetNumberOfPoints(DatasetConstPointerType ds);
            
    static void DeleteDataset(DatasetPointerType ds) {}// do nothing, as we are working with smart pointers

private:
    static DatasetPointerType ReadDataset(const char* filename);
    static MaskPointerType ReadMask(const char* filename);
    static void WriteDataset(const char* filename, const ImageType* image);
    static void WriteMask(const char* filename, const MaskType* image);

	DatasetConstPointerType m_reference;
	MaskConstPointerType    m_mask;
	DomainType m_domain;
	std::vector<unsigned> m_mapPointIDToInternalIdx;
};

} // namespace itk

#include "itkImageROIRepresenter.txx"

#endif /* ITK_IMAGE_ROI_REPRESENTER_H_ */
