/*
 * itkVectorImageRepresenterTest.cpp
 *
 *  Created on: May 3, 2012
 *      Author: luethi
 */

#include "vtkPolyDataRepresenter.h"
#include "../representerTests/genericRepresenterTest.hxx"
#include "statismo/DataManager.h"
#include "statismo/PosteriorModelBuilder.h"
#include "statismo/CommonTypes.h"
#include "statismo/Domain.h"

#include "vtkMath.h"

double distanceBetweenPoints(const vtkPoint& p1, const vtkPoint& p2) {



}

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
	const std::string testDatasetFilename2 = datadir + "/hand_polydata/hand-2.vtk";


	typedef vtkPolyDataRepresenter RepresenterType;
  typedef statismo::DataManager<RepresenterType> DataManagerType;
  typedef vtkPolyDataRepresenter::PointType PointType;
  typedef vtkPolyDataRepresenter::DomainType DomainType;
  typedef DomainType::DomainPointsListType DomainPointsListType;
  typedef statismo::StatisticalModel<RepresenterType> StatisticalModelType;

  typedef statismo::PosteriorModelBuilder<RepresenterType> PosteriorModelBuilderType;
	typedef typename PosteriorModelBuilderType::PointValuePairType PointValuePairType;
	typedef typename PosteriorModelBuilderType::PointValueWithCovariancePairType PointValueWithCovariancePairType;
	typedef typename PosteriorModelBuilderType::PointValueWithCovarianceListType PointValueWithCovarianceListType;
	typedef statismo::MatrixType MatrixType;

  unsigned nPointsFixed = 100;
  unsigned nPointsTest = 1000;
  double tolerance = 0.01;

	vtkPolyData* reference = loadPolyData(referenceFilename);
	RepresenterType* representer = RepresenterType::Create(reference, vtkPolyDataRepresenter::RIGID);

	std::auto_ptr<DataManagerType> dataManager(DataManagerType::Create(representer));

	vtkPolyData* testDataset = loadPolyData(testDatasetFilename);
	vtkPolyData* testDataset2 = loadPolyData(testDatasetFilename2);

	dataManager->AddDataset(reference, "ref");
	dataManager->AddDataset(testDataset, "dataset1");
	dataManager->AddDataset(testDataset2, "dataset2");

	double pointValueNoiseVariance = 0.1;
	const MatrixType pointCovarianceMatrix = pointValueNoiseVariance * MatrixType::Identity(3,3);
	PointValueWithCovarianceListType pvcList;//(pointValues.size());

	RepresenterType::DatasetPointerType testSample = dataManager->GetSampleDataStructure().back()->GetSample();

	DomainPointsListType domaintPointsList = representer->GetDomain().GetDomainPoints();
	unsigned nDomainPoints = representer->GetDomain().GetNumberOfPoints();

	for(unsigned pt_id = 0; pt_id < nDomainPoints;
			pt_id = pt_id + nDomainPoints / nPointsFixed) {
		PointType fixedPoint = domaintPointsList[pt_id];
		PointType valuePoint = testSample->GetPoint(pt_id);
		PointValuePairType pvPair(fixedPoint,valuePoint);
		PointValueWithCovariancePairType pvcPair(pvPair,pointCovarianceMatrix);
		pvcList.push_back(pvcPair);
	}

	PosteriorModelBuilderType* pModelBuilder = PosteriorModelBuilderType::Create();
	StatisticalModelType* posteriorModel = pModelBuilder->BuildNewModel(dataManager->GetSampleDataStructure(),pvcList, 0.1);

	RepresenterType::DatasetPointerType posterior_mean = posteriorModel->DrawMean();

	bool testsOk = true;

	for(unsigned pt_id = 0; pt_id < nDomainPoints;
			pt_id = pt_id + nDomainPoints / nPointsTest) {
		PointType posteriorMeanPoint = posterior_mean->GetPoint(pt_id);
		PointType testPoint = testSample->GetPoint(pt_id);

		double distance2 = vtkMath::Distance2BetweenPoints(posteriorMeanPoint.data(),testPoint.data());
		std::cout << distance2 << std::endl;

		if(distance2 > tolerance * tolerance) testsOk = false;

	}


//	delete representer;
//	reference->Delete();
//	testDataset->Delete();

	if (testsOk == true) {
		std::cout << "Tests passed." << std::endl;
		return EXIT_SUCCESS;
	}
	else {
		std::cout << "Tests failed." << std::endl;
		return EXIT_FAILURE;
	}

}


