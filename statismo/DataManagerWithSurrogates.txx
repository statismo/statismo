/*
 * DataManagerWithSurrogates.txx
 *
 * Created by: Marcel Luethi and  Remi Blanc
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
#include "DataManagerWithSurrogates.h"
#include "HDF5Utils.h"
#include <iostream>

namespace statismo {


////////////////////////////////////////////////
// Data manager With Surrogates
////////////////////////////////////////////////


template <typename Representer>
DataManagerWithSurrogates<Representer>::DataManagerWithSurrogates(const Representer* representer, const std::string& filename)
: DataManager<Representer>(representer)
{
	LoadSurrogateTypes(filename);
}


template <typename Representer>
void
DataManagerWithSurrogates<Representer>::LoadSurrogateTypes(const std::string& filename) {
	VectorType tmpVector;
	tmpVector = Utils::ReadVectorFromTxtFile(filename.c_str());
	for (unsigned i=0 ; i<tmpVector.size() ; i++) {
		if (tmpVector(i)==0) m_surrogateTypes.push_back(SampleDataWithSurrogatesType::Categorical);
		else m_surrogateTypes.push_back(SampleDataWithSurrogatesType::Continuous);
	}
}



template <typename Representer>
void
DataManagerWithSurrogates<Representer>::AddDatasetWithSurrogates(const std::string& datasetFilename,
																 const std::string& surrogateFilename)
{

	assert(this->m_representer != 0);
	assert(this->m_surrogateTypes.size() > 0);

	DatasetPointerType ds = Representer::ReadDataset(datasetFilename.c_str());
	const VectorType& surrogateVector = Utils::ReadVectorFromTxtFile(surrogateFilename.c_str());

	if (surrogateVector.size() != m_surrogateTypes.size() ) throw StatisticalModelException("Trying to loading a dataset with unexpected number of surrogates");

	DatasetPointerType sample = this->m_representer->DatasetToSample(ds, 0);

	this->m_sampleDataList.push_back(SampleDataWithSurrogatesType::Create(this->m_representer,
																		datasetFilename,
																		this->m_representer->SampleToSampleVector(sample),
																	   surrogateFilename,
																	   surrogateVector));
	Representer::DeleteDataset(sample);
	Representer::DeleteDataset(ds);
}


} // Namespace statismo
