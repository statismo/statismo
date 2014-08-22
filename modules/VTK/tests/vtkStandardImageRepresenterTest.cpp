/*
 * itkVectorImageRepresenterTest.cpp
 *
 *  Created on: May 3, 2012
 *      Author: luethi
 */

#include "vtkStandardImageRepresenter.h"
#include "genericRepresenterTest.hxx"


typedef statismo::vtkStandardImageRepresenter<double, 2> RepresenterType;
typedef GenericRepresenterTest< RepresenterType> RepresenterTestType;

vtkStructuredPoints* loadStructuredPoints(const std::string& filename) {
	vtkStructuredPointsReader* reader = vtkStructuredPointsReader::New();
	reader->SetFileName(filename.c_str());
	reader->Update();
	vtkStructuredPoints* pd = vtkStructuredPoints::New();
	pd->ShallowCopy(reader->GetOutput());
	reader->Delete();
	return pd;
}

int main(int argc, char** argv) {

	if (argc < 2) {
		std::cout << "Usage: " << argv[0] << " datadir" << std::endl;
		exit(EXIT_FAILURE);
	}
	std::string datadir = std::string(argv[1]);

	const std::string referenceFilename = datadir + "/hand_dfs/df-hand-1.vtk";
	const std::string testDatasetFilename = datadir + "/hand_dfs/df-hand-2.vtk";

	vtkStructuredPoints* reference = loadStructuredPoints(referenceFilename);

    RepresenterType* representer = RepresenterType::Create(reference);

	// choose a test dataset, a point (on the reference) and the associated point on the test example
	vtkStructuredPoints* testDataset = loadStructuredPoints(testDatasetFilename);
	unsigned testPtId = 0;
    statismo::vtkPoint testPt(reference->GetPoint(testPtId));
    statismo::vtkNDPixel testValue(testDataset->GetPointData()->GetScalars()->GetTuple2(testPtId), 2);
	RepresenterTestType representerTest(representer, testDataset, std::make_pair(testPt, testValue));

	bool testsOk = representerTest.runAllTests();
    delete representer;
    reference->Delete();
    testDataset->Delete();

	if (testsOk == true) {
		return EXIT_SUCCESS;
	}
	else {
		return EXIT_FAILURE;
	}

}


