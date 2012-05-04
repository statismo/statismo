


/*
 * itkVectorImageRepresenterTest.cpp
 *
 *  Created on: May 3, 2012
 *      Author: luethi
 */

#include "itkImageRepresenter.h"
#include "genericRepresenterTest.hxx"



typedef itk::Image< float,2 > ImageType;
typedef itk::ImageRepresenter<float, 2> RepresenterType;

typedef GenericRepresenterTest<RepresenterType> RepresenterTestType;

ImageType::Pointer loadImage(const std::string& filename) {
	itk::ImageFileReader<ImageType>::Pointer reader = itk::ImageFileReader<ImageType>::New();
	reader->SetFileName(filename);
	reader->Update();
	ImageType::Pointer img = reader->GetOutput();
	img->DisconnectPipeline();
	return img;
}

int main(int argc, char** argv) {
	if (argc < 2) {
		std::cout << "Usage: " << argv[0] << " datadir" << std::endl;
		exit(EXIT_FAILURE);
	}
	std::string datadir = std::string(argv[1]);

	const std::string referenceFilename = datadir + "/hand_images/hand-1.vtk";
	const std::string testDatasetFilename = datadir + "/hand_images/hand-2.vtk";

	RepresenterType::Pointer representer = RepresenterType::New();
	ImageType::Pointer reference = loadImage(referenceFilename);
	representer->SetReference(reference);

	// choose a test dataset, a point and its associate pixel value

	ImageType::Pointer testDataset = loadImage(testDatasetFilename);
	ImageType::IndexType idx;
	idx.Fill(0);
	ImageType::PointType testPt;
	reference->TransformIndexToPhysicalPoint(idx, testPt);
	ImageType::PixelType testValue = testDataset->GetPixel(idx);

	RepresenterTestType representerTest(representer, testDataset, std::make_pair(testPt, testValue));

	if (representerTest.runAllTests() == true) {
		return EXIT_SUCCESS;
	}
	else {
		return EXIT_FAILURE;
	}

}


