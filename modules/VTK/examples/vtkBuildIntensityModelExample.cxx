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

#include <boost/scoped_ptr.hpp>

#include <vtkStructuredPoints.h>
#include <vtkStructuredPointsReader.h>

#include "DataManager.h"
#include "PCAModelBuilder.h"
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
// Build a new shape model from vtkPolyData, given in datadir.
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
    typedef DataManager<vtkStructuredPoints> DataManagerType;
    typedef PCAModelBuilder<vtkStructuredPoints> ModelBuilderType;
    typedef StatisticalModel<vtkStructuredPoints> StatisticalModelType;

    try {

        // Model building is exactly the same as for shape models (see BuildShapeModelExample for detailed explanation)
        vtkStructuredPoints* reference = loadVTKStructuredPointsData(datadir +"/hand-0.vtk");
        boost::scoped_ptr<RepresenterType> representer(RepresenterType::Create(reference));
        boost::scoped_ptr<DataManagerType> dataManager(DataManagerType::Create(representer.get()));

        // load the data and add it to the data manager. We take the first 4 hand shapes that we find in the data folder
        for (unsigned i = 0; i < 4; i++) {

            std::ostringstream ss;
            ss << datadir +"/hand-" << i << ".vtk";
            const std::string datasetFilename = ss.str();
            vtkStructuredPoints* dataset = loadVTKStructuredPointsData(datasetFilename);

            // We provde the filename as a second argument.
            // It will be written as metadata, and allows us to more easily figure out what we did later.
            dataManager->AddDataset(dataset, datasetFilename);

            // it is save to delete the dataset after it was added, as the datamanager direclty copies it.
            dataset->Delete();
        }
        boost::scoped_ptr<ModelBuilderType> modelBuilder(ModelBuilderType::Create());
        boost::scoped_ptr<StatisticalModelType> model(modelBuilder->BuildNewModel(dataManager->GetData(), 0.01));
        statismo::IO<vtkStructuredPoints>::SaveStatisticalModel(model.get(), modelname);

        reference->Delete();
        std::cout << "Successfully saved model as " << modelname << std::endl;
    } catch (StatisticalModelException& e) {
        std::cout << "Exception occured while building the intensity model" << std::endl;
        std::cout << e.what() << std::endl;
    }
}
