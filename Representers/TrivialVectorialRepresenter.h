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


#ifndef TRIVIALVECTORIALREPRESENTER_H
#define TRIVIALVECTORIALREPRESENTER_H

#include "statismo/CommonTypes.h"
#include <H5Cpp.h>

// A pointId is actually just an unsigned. However, we need to create a distinct type, to disambiguate some of
// the methods.
struct PointIdType {
		unsigned ptId;

};

/**
 * \brief A trivial representer, that does no representation at all, but works directly with vectorial data
 *
 * \warning This representer is mainly for debugging purposes and not intended to be used for real projets
 */
class TrivialVectorialRepresenter  {
public:



	// For simplicity, we don't use pointers, but copy the objects (as efficiency is not the goal of this representer).
	typedef VectorType DatasetPointerType;
	typedef VectorType DatasetConstPointerType;

	typedef PointIdType PointType;
	typedef statismo::ScalarType ValueType;

	struct DatasetInfo {}; // not used for this representer, but needs to be here as it is part of the generic interface


	static TrivialVectorialRepresenter* Create() {}

	static TrivialVectorialRepresenter* Load(const H5::CommonFG& fg) {return TrivialVectorialRepresenter::Create();}

	TrivialVectorialRepresenter* Clone() const {return TrivialVectorialRepresenter::Create();}
	void Delete() const { delete this; }

	virtual ~TrivialVectorialRepresenter() {}


	static std::string GetName() { return "TrivialVectorialRepresenter"; }
	static unsigned GetDimensions() { return 1; }

	DatasetPointerType DatasetToSample(DatasetConstPointerType ds, DatasetInfo* notUsed) const { return ds; }
	statismo::VectorType SampleToSampleVector(DatasetConstPointerType sample) const { return sample; }
	DatasetPointerType SampleVectorToSample(const statismo::VectorType& sample) const { return sample; }

	statismo::VectorType PointSampleToPointSampleVector(const ValueType& v) const {
		statismo::VectorType vec = statismo::VectorType::Zero(1);
		vec(0) = v;
		return vec;

	}
	ValueType PointSampleVectorToPointSample(const statismo::VectorType& pointSample) const {return pointSample(0); }


	void Save(const H5::CommonFG& fg) const {}
	unsigned GetPointIdForPoint(const PointType& point) const {return point.ptId; }

    
    static void DeleteDataset(DatasetPointerType d) {}

    static unsigned MapPointIdToInternalIdx(unsigned ptId, unsigned componentInd) { return ptId; }

};


#endif
