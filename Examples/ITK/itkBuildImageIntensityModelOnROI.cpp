///*
// * This file is part of the statismo library.
// *
// * Author: Remi Blanc (rblanc33@gmail.com)
// *
// * Copyright (c) 2012 University of Lyon
// * All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without
// * modification, are permitted provided that the following conditions
// * are met:
// *
// * Redistributions of source code must retain the above copyright notice,
// * this list of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright
// * notice, this list of conditions and the following disclaimer in the
// * documentation and/or other materials provided with the distribution.
// *
// * Neither the name of the project's author nor the names of its
// * contributors may be used to endorse or promote products derived from
// * this software without specific prior written permission.
// *
// * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// *
// */
//
//
//#include "itkVectorImageRepresenter.h"
//#include "itkImageROIRepresenter.h"
//#include "statismo_ITK/itkStatisticalModel.h"
//#include "statismo_ITK/itkPCAModelBuilder.h"
//#include "statismo_ITK/itkDataManager.h"
//#include "itkDirectory.h"
//#include "itkMesh.h"
//#include "itkMeshFileWriter.h"
//#include "itkMeshFileReader.h"
//#include <sys/types.h>
//#include <errno.h>
//#include <iostream>
//
///*
// * This example shows the ITK Wrapping of statismo can be used to build a intensity image model restricted to a ROI.
// */
//
//const unsigned Dimension = 2;
//typedef unsigned char PixelType;
//typedef itk::Image<PixelType, Dimension> ImageType;
//typedef itk::ImageROIRepresenter<PixelType, Dimension> RepresenterType;
//typedef itk::ImageFileReader< ImageType > ImageFileReaderType;
//
//
///*function... might want it in some class?*/
//int getdir (std::string dir, std::vector<std::string> &files, const std::string& extension=".*")
//{
//	itk::Directory::Pointer directory = itk::Directory::New();
//	directory->Load(dir.c_str());
//
//	for (unsigned i = 0; i < directory->GetNumberOfFiles(); i++) {
//		const char* filename = directory->GetFile(i);
//		if (extension == ".*" || std::string(filename).find(extension) != std::string::npos)
//            files.push_back(filename);
//	}
//
//    return 0;
//}
//
///*
// * Reads input images
//*/
//
//ImageType::Pointer ReadImageFromFile(const std::string& filename) {
//	ImageFileReaderType::Pointer reader = ImageFileReaderType::New();
//	reader->SetFileName(filename.c_str());
//	reader->Update();
//	return reader->GetOutput();
//}
//
//
//
//void buildImageIntensityModelOnROI(const char* referenceFilename, const char* maskFilename, const char* dir, const char* outputImageFilename) {
//
//
//	typedef itk::PCAModelBuilder<RepresenterType> ModelBuilderType;
//	typedef itk::StatisticalModel<RepresenterType> StatisticalModelType;
//    typedef std::vector<std::string> StringVectorType;
//    typedef itk::DataManager<RepresenterType> DataManagerType;
//
//    RepresenterType::Pointer representer = RepresenterType::New();
//
//	typedef itk::ImageFileReader< ImageType > MaskReaderType;
//	MaskReaderType::Pointer maskReader = MaskReaderType::New();
//	maskReader->SetFileName( maskFilename );
//    maskReader->Update();
//
//	representer->SetReference( ReadImageFromFile(referenceFilename), maskReader->GetOutput() );
//
//    StringVectorType filenames;
//    getdir(dir, filenames, ".vtk");
//
//    DataManagerType::Pointer dataManager = DataManagerType::New();
//	dataManager->SetRepresenter(representer);
//
//    for (StringVectorType::const_iterator it = filenames.begin(); it != filenames.end(); it++) {
//        std::string fullpath = (std::string(dir) + "/") + *it;
//
//        dataManager->AddDataset( ReadImageFromFile(fullpath), fullpath.c_str());
//    }
//
//	ModelBuilderType::Pointer pcaModelBuilder = ModelBuilderType::New();
//    StatisticalModelType::Pointer model = pcaModelBuilder->BuildNewModel(dataManager->GetSampleDataStructure(), 0);
//
//    std::cout<<"dimensionality of the data: "<<model->GetDomain().GetNumberOfPoints()<<", dimension of the images: "<<(*dataManager->GetSampleDataStructure().begin())->GetSample()->GetLargestPossibleRegion().GetNumberOfPixels()<<std::endl;
//
//    std::cout<<"writing the mean sample to a png file..."<<std::endl;
//
//	typedef itk::ImageFileWriter< ImageType > ImageWriterType;
//	ImageWriterType::Pointer writer = ImageWriterType::New();
//	writer->SetFileName( outputImageFilename );
//	writer->SetInput(model->DrawSample());
//    writer->Update();
//
//}
//
//int main(int argc, char* argv[]) {
//
//	if (argc < 5) {
//		std::cout << "usage " << argv[0] << " referenceShape ROIFilename shapeDir outputImage" << std::endl;
//		exit(-1);
//	}
//
//	const char* reference = argv[1];
//	const char* maskFilename = argv[2];
//	const char* dir = argv[3];
//	const char* outputImageFilename = argv[4];
//
//	buildImageIntensityModelOnROI(reference, maskFilename, dir, outputImageFilename);
//
//	std::cout << "Model building is completed successfully." << std::endl;
//}
//
