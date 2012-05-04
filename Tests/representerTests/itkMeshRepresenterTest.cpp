/*
 * itkVectorImageRepresenterTest.cpp
 *
 *  Created on: May 3, 2012
 *      Author: luethi
 */

#include "itkMeshRepresenter.h"
#include "itkMeshFileReader.h"
#include "genericRepresenterTest.hxx"



const unsigned Dimensions = 3;
typedef itk::Mesh<float, Dimensions  > MeshType;
typedef itk::MeshRepresenter<float, Dimensions> RepresenterType;


typedef GenericRepresenterTest<RepresenterType> RepresenterTestType;

MeshType::Pointer loadMesh(const std::string& filename) {
	itk::MeshFileReader<MeshType>::Pointer reader = itk::MeshFileReader<MeshType>::New();
	reader->SetFileName(filename);
	reader->Update();
	MeshType::Pointer mesh = reader->GetOutput();
	mesh->DisconnectPipeline();
	return mesh;
}

int main(int argc, char** argv) {
	if (argc < 2) {
		std::cout << "Usage: " << argv[0] << " datadir" << std::endl;
		exit(EXIT_FAILURE);
	}
	std::string datadir = std::string(argv[1]);

	const std::string referenceFilename = datadir + "/hand_polydata/hand-0.vtk";
	const std::string testDatasetFilename = datadir + "/hand_polydata/hand-1.vtk";

	RepresenterType::Pointer representer = RepresenterType::New();
	MeshType::Pointer reference = loadMesh(referenceFilename);
	representer->SetReference(reference);

	// choose a test dataset, a point and its associate pixel value

	MeshType::Pointer testDataset = loadMesh(testDatasetFilename);
	unsigned testPtId = 0;
	MeshType::PointType testPt;
	reference->GetPoint(testPtId, &testPt);
	MeshType::PointType testValue;
	testDataset->GetPoint(testPtId, &testValue);

	RepresenterTestType representerTest(representer, testDataset, std::make_pair(testPt, testValue));

	if (representerTest.runAllTests() == true) {
		return EXIT_SUCCESS;
	}
	else {
		return EXIT_FAILURE;
	}

}


