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
#include <itkImageFileReader.h>

#include "itkDataManager.h"
#include "itkPCAModelBuilder.h"
#include "itkStandardImageRepresenter.h"
#include "itkStatismoIO.h"
#include "itkStatisticalModel.h"


/*
 * This example shows the ITK Wrapping of statismo can be used to build a deformation model.
 */


typedef itk::Image<float, 3> ImageType3D;
typedef itk::Image< itk::Vector<float, 3> ,3 > VectorImageType3D;
typedef itk::StandardImageRepresenter<itk::Vector<float, 3>, 3> RepresenterType3D;

typedef itk::Image<float, 2> ImageType2D;
typedef itk::Image< itk::Vector<float, 2> ,2 > VectorImageType2D;
typedef itk::StandardImageRepresenter<itk::Vector<float, 2>, 2> RepresenterType2D;



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



template <class RepresenterType, class ImageType>
void itkExample(const char* dir, const char* modelname, double noiseVariance) {



    typedef itk::PCAModelBuilder<ImageType> ModelBuilderType;
    typedef itk::StatisticalModel<ImageType> StatisticalModelType;
    typedef std::vector<std::string> StringVectorType;
    typedef itk::DataManager<ImageType> DataManagerType;
    typedef itk::ImageFileReader<ImageType> ImageFileReaderType;


    StringVectorType filenames;
    getdir(dir, filenames, ".vtk");

    // we take an arbitrary dataset as the reference, as they have all the same resolution anyway
    std::string referenceFilename = (std::string(dir) + "/" + filenames[0]);
    typename ImageFileReaderType::Pointer refReader = ImageFileReaderType::New();
    refReader->SetFileName(referenceFilename);
    refReader->Update();

    typename RepresenterType::Pointer representer = RepresenterType::New();
    representer->SetReference(refReader->GetOutput());


    typename DataManagerType::Pointer dataManager = DataManagerType::New();
    dataManager->SetRepresenter(representer);

    for (StringVectorType::const_iterator it = filenames.begin(); it != filenames.end(); it++) {

        std::string fullpath = (std::string(dir) + "/") + *it;
        typename ImageFileReaderType::Pointer reader = ImageFileReaderType::New();
        reader->SetFileName(fullpath);
        reader->Update();
        typename ImageType::Pointer df = reader->GetOutput();

        dataManager->AddDataset(df, fullpath.c_str());
    }

    typename ModelBuilderType::Pointer pcaModelBuilder = ModelBuilderType::New();
    typename StatisticalModelType::Pointer model = pcaModelBuilder->BuildNewModel(dataManager->GetData(), noiseVariance);
    itk::StatismoIO<ImageType>::SaveStatisticalModel(model, modelname);

}

int main(int argc, char* argv[]) {

    if (argc < 4) {
        std::cout << "usage " << argv[0] << " dimension deformationFieldDir modelname [noiseVariance = 0]" << std::endl;
        exit(-1);
    }

    unsigned int dimension = atoi(argv[1]);
    const char* dir = argv[2];
    const char* modelname = argv[3];

    double noiseVariance = 0;
    if (argc > 4) {
        noiseVariance = atof(argv[4]);
    }

    if (dimension==2) {
        itkExample<RepresenterType2D, VectorImageType2D>(dir, modelname, noiseVariance);
    } else if (dimension==3) {
        itkExample<RepresenterType3D, VectorImageType3D>(dir, modelname, noiseVariance);
    } else {
        assert(0);
    }

    std::cout << "Model building is completed successfully with a noise variance of " << noiseVariance << "." << std::endl;
}

