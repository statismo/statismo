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



#ifndef MODELINFO_H_
#define MODELINFO_H_

#include "H5Cpp.h"
#include "CommonTypes.h"
#include <memory>


namespace statismo {



/**
 * \brief stores information about datasets and parameters that were used to build a model.
 *
 */
class ModelInfo {
public:
	typedef std::pair<std::string, std::string> KeyValuePair;
	typedef std::list<KeyValuePair> KeyValueList;

	// Currently all the info entries are just simple list of string pairs.
	typedef KeyValueList DataInfoList;
	typedef KeyValueList BuilderInfoList;

	/// create an new, empty model info object
	ModelInfo()
	{}

	/**
	 * Creates a new ModelInfo object with the given information
	 * \param scores A matrix holding the scores
	 */
	ModelInfo(const MatrixType& scores, const DataInfoList& di, const BuilderInfoList& bi)
	: m_scores(scores), m_dataInfo(di), m_builderInfo(bi)
	{}

	/// destructor
	virtual ~ModelInfo() {}

	ModelInfo& operator=(const ModelInfo& rhs) {
		if (this == &rhs) {
			return *this;
		}
		this->m_scores = rhs.m_scores;
		this->m_dataInfo = rhs.m_dataInfo;
		this->m_builderInfo = rhs.m_builderInfo;
		return *this;
	}

	ModelInfo(const ModelInfo& orig) {
		operator=(orig);
	}


	/**
	 * Returns the scores matrix. That is, a matrix where the i-th column corresponds to the
	 * coefficients of the i-th dataset in the model
	 */
	const MatrixType& GetScoresMatrix() const { return m_scores; }

	/**
	 * Saves the model info to the given group in the HDF5 file
	 */
	virtual void Save(const H5::CommonFG& publicFg) const;

	/**
	 * Loads the model info from the given group in the HDF5 file.
	 */
	virtual void Load(const H5::CommonFG& publicFg);
	
	/**
	 * Returns the data info
	 */
	const DataInfoList& GetDataInfo() const { return m_dataInfo; }

private:

	void FillKeyValueListFromInfoGroup(const H5::CommonFG& group, KeyValueList& keyValueList);

	MatrixType m_scores;
	DataInfoList m_dataInfo;
	BuilderInfoList m_builderInfo;
};



} // namespace statismo

#include "ModelInfo.cxx"

#endif /* MODELINFO_H_ */
