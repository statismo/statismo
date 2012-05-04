/*
 * itkVectorImageRepresenterTest.cpp
 *
 *  Created on: May 3, 2012
 *      Author: luethi
 */

#include "vtkPolyDataRepresenter.h"
#include "genericRepresenterTest.hxx"



typedef GenericRepresenterTest<vtkPolyDataRepresenter> RepresenterTestType;

vtkPolyData* loadPolyData(const std::string& filename) {
	vtkPolyDataReader* reader = vtkPolyDataReader::New();
	reader->SetFileName(filename.c_str());
	reader->Update();
	vtkPolyData* pd = vtkPolyData::New();
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

	const std::string referenceFilename = datadir + "/hand_polydata/hand-0.vtk";
	const std::string testDatasetFilename = datadir + "/hand_polydata/hand-1.vtk";

	vtkPolyData* reference = loadPolyData(referenceFilename);
	vtkPolyDataRepresenter* representer = vtkPolyDataRepresenter::Create(reference, vtkPolyDataRepresenter::NONE);

	// choose a test dataset, a point (on the reference) and the associated point on the test example

	vtkPolyData* testDataset = loadPolyData(testDatasetFilename);
	unsigned testPtId = 0;
	vtkPoint testPt(reference->GetPoints()->GetPoint(testPtId));
	vtkPoint testValue(testDataset->GetPoints()->GetPoint(testPtId));

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


