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

#include <sys/types.h>
#include <errno.h>
#include <iostream>

#include <itkDirectory.h>
#include <itkMesh.h>
#include <itkMeshFileWriter.h>
#include <itkMeshFileReader.h>

#include "itkDataManager.h"
#include "itkPCAModelBuilder.h"
#include "itkStandardMeshRepresenter.h"
#include "itkStatismoIO.h"
#include "itkStatisticalModel.h"

/*
 * This example shows the ITK Wrapping of statismo can be used to build a shape model.
 */

const unsigned Dimensions = 3;
typedef itk::Mesh<float, Dimensions  > MeshType;

typedef itk::StandardMeshRepresenter<float, Dimensions> RepresenterType;


/*function... might want it in some class?*/
int getdir (std::string dir, std::vector<std::string> &files, const std::string& extension=".*") {
    itk::Directory::Pointer directory = itk::Directory::New();
    directory->Load(dir.c_str());

    for (unsigned i = 0; i < directory->GetNumberOfFiles(); i++) {
        const char* filename = directory->GetFile(i);
        if (extension == ".*" || std::string(filename).find(extension) != std::string::npos)
            files.push_back(filename);
    }

    return 0;
}




void buildShapeModel(const char* referenceFilename, const char* dir, const char* modelname) {


    typedef itk::PCAModelBuilder<MeshType> ModelBuilderType;
    typedef itk::StatisticalModel<MeshType> StatisticalModelType;
    typedef std::vector<std::string> StringVectorType;
    typedef itk::DataManager<MeshType> DataManagerType;
    typedef itk::MeshFileReader<MeshType> MeshReaderType;

    RepresenterType::Pointer representer = RepresenterType::New();

    MeshReaderType::Pointer refReader = MeshReaderType::New();
    refReader->SetFileName(referenceFilename);
    refReader->Update();
    representer->SetReference(refReader->GetOutput());

    StringVectorType filenames;
    getdir(dir, filenames, ".vtk");

    DataManagerType::Pointer dataManager = DataManagerType::New();
    dataManager->SetRepresenter(representer);

    for (StringVectorType::const_iterator it = filenames.begin(); it != filenames.end(); it++) {
        std::string fullpath = (std::string(dir) + "/") + *it;

        MeshReaderType::Pointer reader = MeshReaderType::New();
        reader->SetFileName(fullpath.c_str());
        reader->Update();
        MeshType::Pointer mesh = reader->GetOutput();
        dataManager->AddDataset(mesh, fullpath.c_str());
    }

    ModelBuilderType::Pointer pcaModelBuilder = ModelBuilderType::New();
    StatisticalModelType::Pointer model = pcaModelBuilder->BuildNewModel(dataManager->GetData(), 0);
    itk::StatismoIO<MeshType>::SaveStatisticalModel(model, modelname);



}

int main(int argc, char* argv[]) {

    if (argc < 4) {
        std::cout << "usage " << argv[0] << " referenceShape shapeDir modelname" << std::endl;
        exit(-1);
    }

    const char* reference = argv[1];
    const char* dir = argv[2];
    const char* modelname = argv[3];

    buildShapeModel(reference, dir, modelname);

    std::cout << "Model building is completed successfully." << std::endl;
}

