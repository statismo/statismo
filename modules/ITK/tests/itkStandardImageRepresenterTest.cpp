


/*
 * itkVectorImageRepresenterTest.cpp
 *
 *  Created on: May 3, 2012
 *      Author: luethi
 */

#include "itkStandardImageRepresenter.h"
#include "genericRepresenterTest.hxx"



typedef itk::Image< float,2 > ScalarImageType;


ScalarImageType::Pointer loadScalarImage(const std::string& filename) {
    itk::ImageFileReader<ScalarImageType>::Pointer reader = itk::ImageFileReader<ScalarImageType>::New();
	reader->SetFileName(filename);
	reader->Update();
    ScalarImageType::Pointer img = reader->GetOutput();
	img->DisconnectPipeline();
	return img;
}

int testRepresenterForScalarImage(const std::string& datadir) {
    typedef itk::StandardImageRepresenter<float, 2> RepresenterType;
    typedef GenericRepresenterTest<RepresenterType> RepresenterTestType;

    const std::string referenceFilename = datadir + "/hand_images/hand-1.vtk";
    const std::string testDatasetFilename = datadir + "/hand_images/hand-2.vtk";

    RepresenterType::Pointer representer = RepresenterType::New();
    ScalarImageType::Pointer reference = loadScalarImage(referenceFilename);
    representer->SetReference(reference);

    // choose a test dataset, a point and its associate pixel value

    ScalarImageType::Pointer testDataset = loadScalarImage(testDatasetFilename);
    ScalarImageType::IndexType idx;
    idx.Fill(0);
    ScalarImageType::PointType testPt;
    reference->TransformIndexToPhysicalPoint(idx, testPt);
    ScalarImageType::PixelType testValue = testDataset->GetPixel(idx);

    RepresenterTestType representerTest(representer, testDataset, std::make_pair(testPt, testValue));

    return (representerTest.runAllTests() == true);
}


typedef itk::Image< itk::Vector<float,2>, 2 > VectorImageType;

VectorImageType::Pointer loadVectorImage(const std::string& filename) {
    itk::ImageFileReader<VectorImageType>::Pointer reader = itk::ImageFileReader<VectorImageType>::New();
    reader->SetFileName(filename);
    reader->Update();
    VectorImageType::Pointer img = reader->GetOutput();
    img->DisconnectPipeline();
    return img;
}

int testRepresenterForVectorImage(const std::string& datadir) {

    typedef itk::StandardImageRepresenter<itk::Vector<float, 2>, 2> RepresenterType;
    typedef GenericRepresenterTest<RepresenterType> RepresenterTestType;

    const std::string referenceFilename = datadir + "/hand_dfs/df-hand-1.vtk";
    const std::string testDatasetFilename = datadir + "/hand_dfs/df-hand-2.vtk";

    RepresenterType::Pointer representer = RepresenterType::New();
    VectorImageType::Pointer reference = loadVectorImage(referenceFilename);
    representer->SetReference(reference);

    // choose a test dataset, a point and its associate pixel value

    VectorImageType::Pointer testDataset = loadVectorImage(testDatasetFilename);
    VectorImageType::IndexType idx;
    idx.Fill(0);
    VectorImageType::PointType testPt;
    reference->TransformIndexToPhysicalPoint(idx, testPt);
    VectorImageType::PixelType testValue = testDataset->GetPixel(idx);

    RepresenterTestType representerTest(representer, testDataset, std::make_pair(testPt, testValue));

    return (representerTest.runAllTests() == true);
}


int main(int argc, char** argv) {
	if (argc < 2) {
		std::cout << "Usage: " << argv[0] << " datadir" << std::endl;
		exit(EXIT_FAILURE);
	}
	std::string datadir = std::string(argv[1]);
    bool resultScalarImage = testRepresenterForScalarImage(datadir);
    bool resultVectorImage = testRepresenterForVectorImage(datadir);

    if (resultScalarImage == true && resultVectorImage == true) {
        return EXIT_SUCCESS;
    }
    else {
        return EXIT_FAILURE;
    }
}


