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


#include "statismo/PCAModelBuilder.h"
#include "statismo/StatisticalModel.h"
#include "statismo/DataManager.h"

#include "Representers/VTK/vtkPolyDataRepresenter.h"

#include <iostream>
#include <memory>


using namespace statismo;
using std::auto_ptr;


// illustrates the crossvalidation functionality of the data manager
int main(int argc, char** argv) {

	if (argc < 3) {
		std::cout << "Usage " << argv[0] << " datadir resultdir" << std::endl;
		exit(-1);
	}
	std::string datadir(argv[1]);
	std::string resultdir(argv[2]);


	// All the statismo classes have to be parameterized with the RepresenterType.
	typedef vtkPolyDataRepresenter RepresenterType;
	typedef DataManager<RepresenterType> DataManagerType;
	typedef StatisticalModel<RepresenterType> StatisticalModelType;
	typedef PCAModelBuilder<RepresenterType> ModelBuilderType;
	typedef DataManagerType::CrossValidationFoldListType CVFoldListType;
	typedef DataManagerType::SampleDataListType SampleDataListType;


	try {
		auto_ptr<RepresenterType> representer(RepresenterType::Create(datadir +"/hand_polydata/hand-0.vtk",RepresenterType::RIGID));

		// create a data manager and add a number of datasets for model building
		auto_ptr<DataManagerType> dataManager(DataManagerType::Create(representer.get()));
		dataManager->AddDataset(datadir +"/hand_polydata/hand-1.vtk");
		dataManager->AddDataset(datadir +"/hand_polydata/hand-2.vtk");
		dataManager->AddDataset(datadir +"/hand_polydata/hand-3.vtk");
		dataManager->AddDataset(datadir +"/hand_polydata/hand-5.vtk");
		dataManager->AddDataset(datadir +"/hand_polydata/hand-6.vtk");
		dataManager->AddDataset(datadir +"/hand_polydata/hand-7.vtk");
		dataManager->AddDataset(datadir +"/hand_polydata/hand-8.vtk");
		dataManager->AddDataset(datadir +"/hand_polydata/hand-9.vtk");

		std::cout << "succesfully loaded "<< dataManager->GetNumberOfSamples() << " samples "<< std::endl;

		// create the model builder
		auto_ptr<ModelBuilderType> pcaModelBuilder(ModelBuilderType::Create());

		// We perform 4-fold cross validation
		CVFoldListType cvFoldList = dataManager->GetCrossValidationFolds(4, true);

		// the CVFoldListType is a standard stl list over which we can iterate to get all the folds
		for (CVFoldListType::const_iterator it = cvFoldList.begin();
			it != cvFoldList.end();
			++it)
		{
			// build the model as usual
			auto_ptr<StatisticalModelType> model(pcaModelBuilder->BuildNewModel(it->GetTrainingData(), 0.01));
			std::cout << "built model with  " << model->GetNumberOfPrincipalComponents() << " principal components"<< std::endl;

			// Now we can iterate over the test data and do whatever validation we would like to do.
			const SampleDataListType testSamplesList = it->GetTestingData();

			for (SampleDataListType::const_iterator it = testSamplesList.begin();
					it != testSamplesList.end();
					++it)
			{
				vtkPolyData* testSample = (*it)->GetAsNewSample();
				std::cout << "probability of test sample under the model: " << model->ComputeProbabilityOfDataset(testSample) << std::endl;

				// We are responsible for deleting the sample.
				testSample->Delete();
			}
		}

	}
	catch (StatisticalModelException& e) {
		std::cout << "Exception occured while building the shape model" << std::endl;
		std::cout << e.what() << std::endl;
	}
}

