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

#include <Eigen/QR>
#include "vtkMath.h"
#include "vtkPolyData.h"
#include "vtkPointData.h"
#include "vtkDataArray.h"
#include "vtkPolyDataWriter.h"
#include "vtkSmartPointer.h"
#include "vtkVersion.h"

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

void writePolyData(vtkPolyData* pd, const std::string& filename) {
	vtkSmartPointer< vtkPolyDataWriter > writer = vtkSmartPointer< vtkPolyDataWriter >::New();
	writer->SetFileName(filename.c_str());
#if (VTK_MAJOR_VERSION == 5 )
     writer->SetInput(pd);
#else
	 writer->SetInputData(pd);
#endif
	writer->Update();
}



int main(int argc, char** argv) {
	if (argc < 2) {
		std::cout << "Usage: " << argv[0] << " output_filename" << std::endl;
		exit(EXIT_FAILURE);
	}
	std::string output_filename = std::string(argv[1]);

	const std::string modelFilename = "/export/madagascar/tom/data/femur/model/surface_model.hd5";

	typedef vtkPolyDataRepresenter RepresenterType;
  typedef statismo::DataManager<RepresenterType> DataManagerType;
  typedef vtkPolyDataRepresenter::PointType PointType;
  typedef vtkPolyDataRepresenter::DomainType DomainType;
  typedef DomainType::DomainPointsListType DomainPointsListType;
  typedef statismo::StatisticalModel<RepresenterType> StatisticalModelType;

  typedef statismo::PosteriorModelBuilder<RepresenterType> PosteriorModelBuilderType;
	typedef  PosteriorModelBuilderType::PointValuePairType PointValuePairType;
	typedef  PosteriorModelBuilderType::PointValueWithCovariancePairType PointValueWithCovariancePairType;
	typedef  PosteriorModelBuilderType::PointValueWithCovarianceListType PointValueWithCovarianceListType;
	typedef statismo::MatrixType MatrixType;
	typedef statismo::VectorType VectorType;
	typedef statismo::VectorTypeDoublePrecision VectorTypeDoublePrecision;

  StatisticalModelType* statisticalModel = StatisticalModelType::Load(modelFilename);

  PointType middleFiducial(6.5,-13.5,-4.6);
  PointType funkyTargetPoint(3.6, -23.8, -37.4);


  const RepresenterType* representer = statisticalModel->GetRepresenter();
  vtkPolyData* reference = const_cast<vtkPolyData*> (representer->GetReference());

  unsigned idOfFiducial = representer->GetPointIdForPoint(middleFiducial);
  double normalOfFiducial[3];
  reference->GetPointData()->GetNormals()->GetTuple(idOfFiducial,normalOfFiducial);


  Eigen::Map<VectorTypeDoublePrecision> normalVector(normalOfFiducial,3);



	// The QR decomposition makes a matrix of the normal vector, together with two
	// tangential vectors.
	typedef Eigen::HouseholderQR<statismo::MatrixType> QRType;
	QRType QR(3,3);
	QR.compute(normalVector.cast<float>());
	statismo::MatrixType normalMatrix = QR.householderQ();



  statismo::VectorType anisotropicNoiseVariances(3); anisotropicNoiseVariances << .01, 100, 100;


	MatrixType fiducialCovMatrix = normalMatrix *
			anisotropicNoiseVariances.asDiagonal() * normalMatrix.transpose();

	//std::cout << fiducialCovMatrix << std::endl;
	//std::cout << idOfFiducial << std::endl;

	PointValuePairType pvPair(middleFiducial,funkyTargetPoint);
	PointValueWithCovariancePairType pvcPair(pvPair, fiducialCovMatrix);
	PointValueWithCovarianceListType pvcList;
	pvcList.push_back(pvcPair);

	PointType upperPoint(-21, 19.8, -261.2);
	PointType lowerPoint(-22.5, 26.2, 260.8);

	PointValueWithCovariancePairType upperPair(PointValuePairType(upperPoint,upperPoint),MatrixType::Identity(3,3));
	PointValueWithCovariancePairType lowerPair(PointValuePairType(lowerPoint,lowerPoint),MatrixType::Identity(3,3));

	pvcList.push_back(upperPair);
	pvcList.push_back(lowerPair);

	//std::cout << "Building posterior model..." << std::endl;
	PosteriorModelBuilderType* onePointPosteriorModelBuilder = PosteriorModelBuilderType::Create();
	StatisticalModelType* onePointPosteriorModel
		= onePointPosteriorModelBuilder->BuildNewModelFromModel(statisticalModel, pvcList, false);
	//std::cout << "... Done." << std::endl;

	vtkPolyData* posteriorMean = onePointPosteriorModel->DrawMean();

	double probability = statisticalModel->ComputeLogProbabilityOfDataset(posteriorMean);
	//std::cout << "Log probability of posterior mean = " << probability << std::endl;

	writePolyData(posteriorMean,output_filename);

	bool testsOk = true;
	if(abs(probability) > 203) {
		testsOk = false;
		std::cout << "The probability within the original model of the posterior mean is too small." << std::endl;
		std::cout << "This means that the anisotropic noise model does not work." << std::endl;
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


