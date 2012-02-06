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


#ifndef __DATAMANAGER_H_
#define __DATAMANAGER_H_


#include "CommonTypes.h"
#include "utils.h"
#include "Exceptions.h"
#include "HDF5Utils.h"
#include "ModelInfo.h"
#include "SampleData.h"

#include <list>

namespace statismo {



/**
 * \brief Holds training and test data used for Crossvalidation
 */
template <typename Representer>
class CrossValidationFold {
public:
	typedef SampleData<Representer> SampleDataType;
	typedef std::list<const SampleDataType*> SampleDataListType;

	/***
	 * Create an empty fold
	 */
	CrossValidationFold() {};

	/**
	 * Create a fold with the given trainingData and testingData
	 */
	CrossValidationFold(const SampleDataListType& trainingData, const SampleDataListType& testingData)
	: m_trainingData(trainingData), m_testingData(testingData)
	{}

	/**
	 * Get a list holding the training data
	 */
	SampleDataListType GetTrainingData() const { return m_trainingData; }

	/**
	 * Get a list holding the testing data
	 */
	SampleDataListType GetTestingData() const { return m_testingData; }


private:
	SampleDataListType m_trainingData;
	SampleDataListType m_testingData;
};


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
class DataManager {

	typedef typename Representer::DatasetPointerType DatasetPointerType;
	typedef typename Representer::DatasetConstPointerType DatasetConstPointerType;


public:
	typedef Representer RepresenterType;
	typedef SampleData<Representer> SampleDataType;
	typedef SampleDataWithSurrogates<Representer> SampleDataWithSurrogatesType;
	typedef std::list<const SampleDataType*> SampleDataListType;
	typedef CrossValidationFold<Representer> CrossValidationFoldType;
	typedef std::list<CrossValidationFoldType> CrossValidationFoldListType;

	/**
	 * Factory method that creates a new instance of a DataManager class
	 *
	 */
	static DataManager<Representer>* Create(const Representer* representer) { return new DataManager<Representer>(representer); }

	/**
	 * Create a new dataManager, with the data stored in the given hdf5 file
	 */
	static DataManager<Representer>* Load(const std::string& filename);


	/**
	 * Destroy the object.
	 * The same effect can be achieved by deleting the object in the usual
	 * way using the c++ delete keyword.
	 */
	void Delete() {delete this; }

	/**
	 * Destructor
	 */
	virtual ~DataManager();


	/**
	 * Add a dataset to the data manager.
	 * \param filename The filename of the dataset.
	 */
	virtual void AddDataset(const std::string& filename);
	
	/**
	 * Saves the data matrix and all URIs into an HDF5 file.
	 * \param filename
	 */
	virtual void Save(const std::string& filename) const;


	/**
	 * return a list with all the sample data objects managed by the data manager
	 */
	SampleDataListType GetSampleData() const;

	/**
	 * returns the number of samples managed by the datamanager
	 */
	unsigned GetNumberOfSamples() const { return m_sampleDataList.size(); }

	/**
	 * Assigns the data to one of n Folds to be used for cross validation.
	 * This method has to be called before cross validation can be started.
	 *
	 * \param nFolds The number of folds used in the crossvalidation
	 * \param randomize If true, the data will be randomly assigned to the nfolds, otherwise the order with which it was added is preserved
	 */
	CrossValidationFoldListType GetCrossValidationFolds(unsigned nFolds, bool randomize = true) const;




protected:
	DataManager(const Representer* representer);

	DataManager(const DataManager& orig);
	DataManager& operator=(const DataManager& rhs);

	Representer* m_representer; // TODO make this a shared pointer

	// members
	SampleDataListType m_sampleDataList;
};

}

#include "DataManager.txx"


#endif /* __DATAMANAGER_H_ */
