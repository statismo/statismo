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

#include <Eigen/Geometry>
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
		if(distance2 > tolerance * tolerance) testsOk = false;

	}


	vtkPolyData* onePointPD = vtkPolyData::New();
	vtkPoints* onePoint = vtkPoints::New();
	onePoint->Allocate(1);
	onePoint->InsertNextPoint(0,0,0);
	onePointPD->SetPoints(onePoint);
	RepresenterType* onePointRepresenter = RepresenterType::Create(onePointPD, vtkPolyDataRepresenter::NONE);
	VectorType onePointMean(3); onePointMean << 0, 0, 0;
	VectorType onePointVar(3);  onePointVar  << 1, 1, 1;
	MatrixType onePointPCABasis = MatrixType::Identity(3,3);

	StatisticalModelType* onePointModel = StatisticalModelType::Create(onePointRepresenter,
																																		onePointMean,
																																		onePointPCABasis,
																																		onePointVar,
																																		0.0);


	Eigen::Matrix3f rotMatrix;
	rotMatrix = Eigen::AngleAxisf(40,Eigen::Vector3f(0,0,1));

	VectorType covTestVector(3);  covTestVector  <<  1,0,0;
	VectorType covTestVector1(3); covTestVector1 <<  0,2,0;
	VectorType covTestVector2(3); covTestVector2 <<  0,0,3;

	MatrixType testMatrix(3,3);
	testMatrix.col(0) = covTestVector;
	testMatrix.col(1) = covTestVector1;
	testMatrix.col(2) = covTestVector2;

	testMatrix = rotMatrix * testMatrix;

	VectorType testValuePoint(3); testValuePoint << 1,1,1;
	testValuePoint = rotMatrix * testValuePoint;
	vtkPoint tvPoint(testValuePoint(0), testValuePoint(1),testValuePoint(2));

	MatrixType testCovMatrix = testMatrix * testMatrix.transpose();
	PointValuePairType pvPair(vtkPoint(0,0,0),tvPoint);
	PointValueWithCovariancePairType pvcPair(pvPair, testCovMatrix);
	PointValueWithCovarianceListType onePointPvcList;
	onePointPvcList.push_back(pvcPair);

	PosteriorModelBuilderType* onePointPosteriorModelBuilder = PosteriorModelBuilderType::Create();
	StatisticalModelType* onePointPosteriorModel
		= onePointPosteriorModelBuilder->BuildNewModelFromModel(onePointModel, onePointPvcList);

	VectorType posteriorModelMean = rotMatrix.inverse() * onePointPosteriorModel->GetMeanVector();
	VectorType knownSolution(3); knownSolution << 0.5, 0.2, 0.1;

	if ((posteriorModelMean - knownSolution).norm() > 0.00001) {
		std::cout << "One point model test failed: Mean not correct." << std::endl;
		testsOk = false;
	}

	VectorType posteriorModelVariance = onePointPosteriorModel->GetPCAVarianceVector();
	VectorType knownVariance(3); knownVariance << 0.9, 0.8, 0.5;
	if ((posteriorModelVariance - knownVariance).norm() > 0.00001) {
		std::cout << "One point model test failed: Variance not correct." << std::endl;
		testsOk = false;
	}

	MatrixType rotatedOrthoPCAMatrix = rotMatrix.inverse() * onePointPosteriorModel->GetOrthonormalPCABasisMatrix();
	if ((rotatedOrthoPCAMatrix * rotatedOrthoPCAMatrix.transpose() - MatrixType::Identity(3,3)).norm() > 0.00001) {
		std::cout << "One point model test failed: PCA basis not correct." << std::endl;
		testsOk = false;
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


