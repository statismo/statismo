/*
 * This file is part of the statismo library.
 *
 * Author: Remi Blanc
 *
 * Copyright (c) 2011, ETH Zurich
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

#include <boost/scoped_ptr.hpp>

#include "ConditionalModelBuilder.h"
#include "DataManager.h"
#include "StatisticalModel.h"
#include "StatismoIO.h"

#include "vtkStandardImageRepresenter.h"

using namespace statismo;


vtkStructuredPoints* loadVTKStructuredPointsData(const std::string& filename) {
    vtkStructuredPointsReader* reader = vtkStructuredPointsReader::New();
    reader->SetFileName(filename.c_str());
    reader->Update();
    vtkStructuredPoints* sp = vtkStructuredPoints::New();
    sp->ShallowCopy(reader->GetOutput());
    return sp;
}

//
// This example shows the use of the classes for
// building an intensity model (appearance model) from
// a number of images.
//
int main(int argc, char** argv) {

    if (argc < 3) {
        std::cout << "Usage " << argv[0] << " datadir modelname" << std::endl;
        exit(-1);
    }
    std::string datadir(argv[1]);
    std::string modelname(argv[2]);


    // All the statismo classes have to be parameterized with the RepresenterType.
    // For building a intensity model with vtk, we use the vtkStructuredPointsRepresenter.
    // Here, we work with unsigned character images. The second template parameter specifies
    // the pixel dimension (1 means scalar image, whereas 3 is a 3D vector image).
    typedef vtkStandardImageRepresenter<unsigned char, 1> RepresenterType;
    typedef DataManagerWithSurrogates<vtkStructuredPoints> DataManagerWithSurrogatesType;
    typedef ConditionalModelBuilder<vtkStructuredPoints> ConditionalModelBuilderType;
    typedef StatisticalModel<vtkStructuredPoints> StatisticalModelType;


    try {
        vtkStructuredPoints* reference = loadVTKStructuredPointsData(datadir +"/hand-0.vtk");
        boost::scoped_ptr<RepresenterType> representer(RepresenterType::Create(reference));


        // We use the SurrogateDataManager, as we need to specify surrogate data in addition to the images.
        // We provide in addition to the representer also a file that contains a description of the surrogate
        // variables (e.g. whether they are categorical or continuous). See the API doc for more details.
        boost::scoped_ptr<DataManagerWithSurrogatesType> dataManager(DataManagerWithSurrogatesType::Create(representer.get(),
                datadir +"/surrogates/hand_surrogates_types.txt"));

        // add the data information. The first argument is the dataset, the second the surrogate information
        // and the 3rd the surrogate type

        // load the data and add it to the data manager. We take the first 4 hand images that we find in the data folder
        for (unsigned i = 0; i < 4; i++) {

            std::ostringstream ssFilename;
            ssFilename << datadir << "/hand-" << i << ".vtk";
            const std::string datasetFilename = ssFilename.str();

            std::ostringstream ssSurrogateFilename;
            ssSurrogateFilename << datadir << "/surrogates/hand-" << i << "_surrogates.txt";
            const std::string surrogateFilename = ssSurrogateFilename.str();

            vtkStructuredPoints* dataset = loadVTKStructuredPointsData(datasetFilename);


            // We provde the filename as a second argument.
            // It will be written as metadata, and allows us to more easily figure out what we did later.
            dataManager->AddDatasetWithSurrogates(dataset, datasetFilename, surrogateFilename);

            // it is save to delete the dataset after it was added, as the datamanager direclty copies it.
            dataset->Delete();
        }

        // Build up a list holding the conditioning information.
        typedef ConditionalModelBuilder<vtkStructuredPoints> ConditionalModelBuilderType;
        ConditionalModelBuilderType::CondVariableValueVectorType conditioningInfo;
        conditioningInfo.push_back(ConditionalModelBuilderType::CondVariableValuePair(true, 1));
        conditioningInfo.push_back(ConditionalModelBuilderType::CondVariableValuePair(false, 65));
        conditioningInfo.push_back(ConditionalModelBuilderType::CondVariableValuePair(true, 86.1));
        conditioningInfo.push_back(ConditionalModelBuilderType::CondVariableValuePair(true, 162.0));

        // Create the model builder and build the model
        boost::scoped_ptr<ConditionalModelBuilderType> modelBuilder(ConditionalModelBuilderType::Create());

        boost::scoped_ptr<StatisticalModelType> model(modelBuilder->BuildNewModel(dataManager->GetData(),
                dataManager->GetSurrogateTypeInfo(),
                conditioningInfo,
                0.1));
        std::cout << "successfully built conditional model" << std::endl;

        // The resulting model is a normal statistical model, from which we could for example sample examples.
        // Here we simply  save it to disk for later use.
        statismo::IO<vtkStructuredPoints>::SaveStatisticalModel(model.get(), modelname);
        reference->Delete();
        std::cout << "save model as " << modelname << std::endl;
    } catch (StatisticalModelException& e) {
        std::cout << "Exception occured while building the conditional model" << std::endl;
        std::cout << e.what() << std::endl;
    }
}
