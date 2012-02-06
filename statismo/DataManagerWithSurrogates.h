/*
 * DataManagerWithSurrogates.h
 *
 * Created by Marcel Luethi and Remi Blanc
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

#ifndef __DATAMANAGERWITHSURROGATES_H_
#define __DATAMANAGERWITHSURROGATES_H_

#include "DataManager.h"

namespace statismo {


/**
 * \brief Manages Training and Test Data for building Statistical Models and provides functionality for Crossvalidation.
 *
 * The DataManager class provides functionality for loading and managing data sets to be used in the
 * statistical model. The datasets are loaded either by using DataManager::AddDataset or directly from a hdf5 File using
 * the Load function. Per default all the datasets are marked as training data. It is, however, often useful
 * to leave a few datasets out to validate the model. For this purpose, the DataManager class implements basic
 * crossvalidation functionality.
 *
 * Internally, the data is kept as a large matrix. This datamatrix is built from the training data,
 * once the DataManager::LoadData method is called.
 */
template <typename Representer>
class DataManagerWithSurrogates : public DataManager<Representer> {

	typedef typename Representer::DatasetPointerType DatasetPointerType;
	typedef typename Representer::DatasetConstPointerType DatasetConstPointerType;


public:
	typedef Representer RepresenterType;
	typedef SampleDataWithSurrogates<Representer> SampleDataWithSurrogatesType;

	typedef typename SampleDataWithSurrogatesType::SurrogateTypeVectorType SurrogateTypeVectorType;

	/**
	 * Destructor
	 */
	virtual ~DataManagerWithSurrogates() {}


	/**
	* Factory method that creates a new instance of a DataManager class
	*
	*/
	static DataManagerWithSurrogates<Representer>* Create(const Representer* representer, const std::string& surrogTypeFilename) {
		return new DataManagerWithSurrogates<Representer>(representer, surrogTypeFilename);
	}


	/**
	 * Add a dataset, together with surrogate information
	 * \param datasetFilename
	 * \param surrogateFilename
	 */
	void AddDatasetWithSurrogates(const std::string& datasetFilename, const std::string& surrogateFilename);

	/**
	 * Get a vector indicating the types of surrogates variables (Categorical vs Continuous)
	 */
	SurrogateTypeVectorType GetSurrogateTypes() const {return m_surrogateTypes;}


protected:

	/**
	 * Loads the information concerning the types of the surrogates variables (categorical=0, continuous=1)
	 * => it is assumed to be in a text file with the entries separated by spaces
	 */
	void LoadSurrogateTypes(const std::string& filename);



	// private - to prevent use
	DataManagerWithSurrogates(const Representer* r, const std::string& filename);

	DataManagerWithSurrogates(const DataManagerWithSurrogates& orig);
	DataManagerWithSurrogates& operator=(const DataManagerWithSurrogates& rhs);

	SurrogateTypeVectorType m_surrogateTypes;
};

}

#include "DataManagerWithSurrogates.txx"


#endif /* __DATAMANAGERWITHSURROGATES_H_ */
