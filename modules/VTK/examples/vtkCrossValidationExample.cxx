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
#include <iostream>
#include <ostream>

#include <vtkPolyDataReader.h>
#include <vtkPolyData.h>

#include "statismo/core/DataManager.h"
#include "statismo/core/PCAModelBuilder.h"
#include "statismo/core/StatisticalModel.h"

#include "statismo/VTK/vtkStandardMeshRepresenter.h"

#include <memory>

using namespace statismo;
using std::unique_ptr;


vtkPolyData *
loadVTKPolyData(const std::string & filename)
{
  vtkPolyDataReader * reader = vtkPolyDataReader::New();
  reader->SetFileName(filename.c_str());
  reader->Update();
  vtkPolyData * pd = vtkPolyData::New();
  pd->ShallowCopy(reader->GetOutput());
  return pd;
}

// illustrates the crossvalidation functionality of the data manager
int
main(int argc, char ** argv)
{

  if (argc < 2)
  {
    std::cout << "Usage " << argv[0] << " datadir" << std::endl;
    exit(-1);
  }
  std::string datadir(argv[1]);


  // All the statismo classes have to be parameterized with the RepresenterType.
  typedef vtkStandardMeshRepresenter                   RepresenterType;
  typedef BasicDataManager<vtkPolyData>                DataManagerType;
  typedef StatisticalModel<vtkPolyData>                StatisticalModelType;
  typedef PCAModelBuilder<vtkPolyData>                 ModelBuilderType;
  typedef DataManagerType::CrossValidationFoldListType CVFoldListType;
  typedef DataManagerType::DataItemListType            DataItemListType;


  try
  {
    vtkPolyData * reference = loadVTKPolyData(datadir + "/hand-0.vtk");
    auto          representer = RepresenterType::SafeCreate(reference);

    // create a data manager and add a number of datasets for model building
    auto dataManager = DataManagerType::SafeCreate(representer.get());

    for (unsigned i = 0; i < 17; i++)
    {

      std::ostringstream ss;
      ss << datadir + "/hand-" << i << ".vtk";
      const std::string datasetFilename = ss.str();
      vtkPolyData *     dataset = loadVTKPolyData(datasetFilename);

      // We provde the filename as a second argument.
      // It will be written as metadata, and allows us to more easily figure out what we did later.
      dataManager->AddDataset(dataset, datasetFilename);

      // it is save to delete the dataset after it was added, as the datamanager direclty copies it.
      dataset->Delete();
    }

    std::cout << "succesfully loaded " << dataManager->GetNumberOfSamples() << " samples " << std::endl;

    // create the model builder
    auto pcaModelBuilder = ModelBuilderType::SafeCreate();

    // We perform 4-fold cross validation
    CVFoldListType cvFoldList = dataManager->GetCrossValidationFolds(4, true);

    // the CVFoldListType is a standard stl list over which we can iterate to get all the folds
    for (CVFoldListType::const_iterator it = cvFoldList.begin(); it != cvFoldList.end(); ++it)
    {
      // build the model as usual
      auto model = pcaModelBuilder->BuildNewModel(it->GetTrainingData(), 0.01);
      std::cout << "built model with  " << model->GetNumberOfPrincipalComponents() << " principal components"
                << std::endl;

      // Now we can iterate over the test data and do whatever validation we would like to do.
      const DataItemListType testSamplesList = it->GetTestingData();

      for (DataItemListType::const_iterator it = testSamplesList.begin(); it != testSamplesList.end(); ++it)
      {
        vtkPolyData * testSample = (*it)->GetSample();
        std::cout << "probability of test sample under the model: " << model->ComputeProbability(testSample)
                  << std::endl;

        // We are responsible for deleting the sample.
        testSample->Delete();
      }
    }
    reference->Delete();
  }
  catch (StatisticalModelException & e)
  {
    std::cout << "Exception occured while building the shape model" << std::endl;
    std::cout << e.what() << std::endl;
  }
}
