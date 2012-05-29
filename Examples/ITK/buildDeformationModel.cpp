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


#include "itkVectorImageRepresenter.h"
#include "itkImageRepresenter.h"
#include "statismo_ITK/itkStatisticalModel.h"
#include "statismo_ITK/itkPCAModelBuilder.h"
#include "statismo_ITK/itkDataManager.h"
#include "itkImageFileReader.h"
#include "itkDirectory.h"
#include <sys/types.h>
#include <errno.h>
#include <iostream>

/*
 * This example shows the ITK Wrapping of statismo can be used to build a deformation model.
 */


typedef itk::Image<float, 3> ImageType3D;
typedef itk::Image< itk::Vector<float, ImageType3D::ImageDimension> , ImageType3D::ImageDimension > VectorImageType3D;
typedef itk::VectorImageRepresenter<float, 3, 3> RepresenterType3D;

typedef itk::Image<float, 2> ImageType2D;
typedef itk::Image< itk::Vector<float, ImageType2D::ImageDimension> , ImageType2D::ImageDimension > VectorImageType2D;

typedef itk::VectorImageRepresenter<float, 2, 2> RepresenterType2D;


/*function... might want it in some class?*/
int getdir (std::string dir, std::vector<std::string> &files, const std::string& extension=".*")
{
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
void itkExample(const char* referenceFilename, const char* dir, const char* modelname) {



	typedef itk::PCAModelBuilder<RepresenterType> ModelBuilderType;
	typedef itk::StatisticalModel<RepresenterType> StatisticalModelType;
    typedef std::vector<std::string> StringVectorType;
    typedef itk::DataManager<RepresenterType> DataManagerType;
	typedef itk::ImageFileReader<ImageType> ImageFileReaderType;

	typename ImageFileReaderType::Pointer refReader = ImageFileReaderType::New();
	refReader->SetFileName(referenceFilename);
	refReader->Update();

    typename RepresenterType::Pointer representer = RepresenterType::New();
    representer->SetReference(refReader->GetOutput());

    StringVectorType filenames;
    getdir(dir, filenames, ".vtk");

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
    typename StatisticalModelType::Pointer model = pcaModelBuilder->BuildNewModel(dataManager->GetSampleData(), 0);
    model->Save(modelname);

}

int main(int argc, char* argv[]) {

	if (argc < 4) {
		std::cout << "usage " << argv[0] << " referenceDeformationField deformationFieldDir modelname" << std::endl;
		exit(-1);
	}

	const char* reference = argv[1];
	const char* dir = argv[2];
	const char* modelname = argv[3];

	itk::ImageIOBase::Pointer imageIO =
	  itk::ImageIOFactory::CreateImageIO(reference, itk::ImageIOFactory::ReadMode);
 
	imageIO->SetFileName(reference);
	imageIO->ReadImageInformation();
	const size_t numDimensions =  imageIO->GetNumberOfDimensions();
    
	if (numDimensions==2){
	  itkExample<RepresenterType2D, VectorImageType2D>(reference, dir, modelname);
	}
	else if (numDimensions==3){
	  itkExample<RepresenterType3D, VectorImageType3D>(reference, dir, modelname);
	}
	else{
	  assert(0);
	}

	std::cout << "Model building is completed successfully." << std::endl;
}

