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



#ifndef ITKVECTORIMAGE_LM_ALIGN_REPRESENTER_H_
#define ITKVECTORIMAGE_LM_ALIGN_REPRESENTER_H_

#include "itkImage.h"
#include "statismo/CommonTypes.h"
#include "itkObject.h"
#include "itkVectorImageRepresenterBase.h"
#include <H5Cpp.h>

/**
 * This class provides a specialization of the Representer for the type itkImage.
 *
 * See Representer for more details about representer classes
 * \sa Representer
 */

namespace itk {

template <class TPixel, unsigned ImageDimension, unsigned VectorDimension>
class VectorImageLMAlignRepresenter : public VectorImageRepresenterBase<TPixel, ImageDimension, VectorDimension> {
public:

	/* Standard class typedefs. */
	typedef VectorImageRepresenterBase<TPixel, ImageDimension, VectorDimension> VectorImageRepresenterBaseType;
	typedef VectorImageLMAlignRepresenter            Self;
	typedef VectorImageRepresenterBaseType	Superclass;
	typedef SmartPointer<Self>                Pointer;
	typedef SmartPointer<const Self>          ConstPointer;


	/** New macro for creation of through a Smart Pointer. */
	itkSimpleNewMacro( Self );

	/** Run-time type information (and related methods). */

	itkTypeMacro( VectorImageLMAlignRepresenter, VectorImageRepresenterBaseType );

	virtual VectorImageLMAlignRepresenter* Clone() const;
	static VectorImageLMAlignRepresenter* Load(const H5::CommonFG& fg);

	void Delete() { this->UnRegister(); }

	static const std::string GetName() { return "itkVectorImageLMAlignRepresenter"; }
	static unsigned GetDimensions() { return Superclass::Dimensions;; }

	// Const correcness is difficult to enforce using smart pointers, as no conversion
	// between nonconst and const pointer are possible. Thus we define both to be non-const
	typedef typename Superclass::DatasetPointerType DatasetPointerType;
	typedef typename Superclass::DatasetConstPointerType DatasetConstPointerType;
	typedef typename Superclass::DatasetType DatasetType;

	typedef typename Superclass::PointType PointType;
	typedef typename Superclass::ValueType ValueType;

	typedef std::vector<PointType> AlignmentListType;

	struct DatasetInfo {}; // not used for this representer, but needs to be here as it is part of the generic interface

	VectorImageLMAlignRepresenter();
	virtual ~VectorImageLMAlignRepresenter();


	void SetAlignmentPoints(const AlignmentListType& alignmentList);

	/**
	 * Creates a sample by first aligning the dataset ds to the reference using Procrustes
	 * Alignment.
	 */
	DatasetPointerType DatasetToSample(DatasetType* ds, DatasetInfo* notUsed) const;
	statismo::VectorType SampleToSampleVector(DatasetType* ds) const;
	DatasetPointerType SampleVectorToSample(const statismo::VectorType& sample) const;

	ValueType PointSampleVectorToPointSample(const statismo::VectorType& pointSample) const;
	statismo::VectorType PointSampleToPointSampleVector(const ValueType& v) const;


	virtual void Save(const H5::CommonFG& fg) const;



private:
	AlignmentListType m_alignmentList;

};

} // namespace itk

#include "itkVectorImageLMAlignRepresenter.txx"

#endif /* itkVectorImageLMAlignRepresenter_H_ */
