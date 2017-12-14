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
#include <boost/scoped_ptr.hpp>

#include <Eigen/Geometry>

#include <vtkMath.h>
#include <vtkPolyDataReader.h>
#include <vtkPolyDataWriter.h>
#include <vtkVersion.h>

#include "CommonTypes.h"
#include "DataManager.h"
#include "Domain.h"
#include "genericRepresenterTest.hxx"
#include "PosteriorModelBuilder.h"
#include "PCAModelBuilder.h"
#include "vtkStandardMeshRepresenter.h"

using namespace statismo;

typedef GenericRepresenterTest<vtkStandardMeshRepresenter> RepresenterTestType;

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
        std::cout << "Usage: " << argv[0] << " datadir" << std::endl;
        exit(EXIT_FAILURE);
    }
    std::string datadir = std::string(argv[1]);

    const std::string referenceFilename = datadir + "/hand_polydata/hand-0.vtk";
    const std::string testDatasetFilename = datadir + "/hand_polydata/hand-1.vtk";
    const std::string testDatasetFilename2 = datadir + "/hand_polydata/hand-2.vtk";
    const std::string testDatasetFilename3 = datadir + "/hand_polydata/hand-3.vtk";
    const std::string testDatasetFilename4 = datadir + "/hand_polydata/hand-4.vtk";


    typedef vtkStandardMeshRepresenter RepresenterType;
    typedef statismo::DataManager<vtkPolyData> DataManagerType;
    typedef vtkStandardMeshRepresenter::PointType PointType;
    typedef vtkStandardMeshRepresenter::DomainType DomainType;
    typedef DomainType::DomainPointsListType DomainPointsListType;
    typedef statismo::StatisticalModel<vtkPolyData> StatisticalModelType;

    typedef statismo::PosteriorModelBuilder<vtkPolyData> PosteriorModelBuilderType;
    typedef  PosteriorModelBuilderType::PointValuePairType PointValuePairType;
    typedef  PosteriorModelBuilderType::PointValueWithCovariancePairType PointValueWithCovariancePairType;
    typedef  PosteriorModelBuilderType::PointValueWithCovarianceListType PointValueWithCovarianceListType;
    typedef statismo::MatrixType MatrixType;

    unsigned nPointsFixed = 100;
    unsigned nPointsTest = 1000;
    double tolerance = 0.01;

    vtkPolyData* reference = loadPolyData(referenceFilename);
    RepresenterType* representer = RepresenterType::Create(reference);

    boost::scoped_ptr<DataManagerType> dataManager(DataManagerType::Create(representer));

    vtkPolyData* testDataset = loadPolyData(testDatasetFilename);
    vtkPolyData* testDataset2 = loadPolyData(testDatasetFilename2);
    vtkPolyData* testDataset3 = loadPolyData(testDatasetFilename3);
    vtkPolyData* testDataset4 = loadPolyData(testDatasetFilename4);


    dataManager->AddDataset(reference, "ref");
    dataManager->AddDataset(testDataset, "dataset1");
    dataManager->AddDataset(testDataset2, "dataset2");
    dataManager->AddDataset(testDataset3, "dataset3");
    dataManager->AddDataset(testDataset4, "dataset4");

    double pointValueNoiseVariance = 0.1;
    const MatrixType pointCovarianceMatrix = pointValueNoiseVariance * MatrixType::Identity(3,3);
    PointValueWithCovarianceListType pvcList;//(pointValues.size());

    RepresenterType::DatasetPointerType testSample = dataManager->GetData().back()->GetSample();

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
    StatisticalModelType* posteriorModel = pModelBuilder->BuildNewModel(dataManager->GetData(),pvcList, 0.1);

    RepresenterType::DatasetPointerType posterior_mean = posteriorModel->DrawMean();

    bool testsOk = true;

    for(unsigned pt_id = 0; pt_id < nDomainPoints;
            pt_id = pt_id + nDomainPoints / nPointsTest) {
        PointType posteriorMeanPoint = posterior_mean->GetPoint(pt_id);
        PointType testPoint = testSample->GetPoint(pt_id);

        double distance2 = vtkMath::Distance2BetweenPoints(posteriorMeanPoint.data(),testPoint.data());
        if(distance2 > tolerance * tolerance) {
            std::cout << "Hand model test failed: Posterior mean not correct." << std::endl;
            testsOk = false;
        }
    }


    typedef statismo::PCAModelBuilder<vtkPolyData> PCAModelBuilderType;
    PCAModelBuilderType* pcaModelBuilder = PCAModelBuilderType::Create();
    StatisticalModelType* fullModel = pcaModelBuilder->BuildNewModel(dataManager->GetData(),0.1,false);


    PointType middleFiducial(244, 290, 0);
    PointType funkyTargetPoint(270, 300, 0);

    statismo::VectorType anisotropicNoiseVariances(3);
    anisotropicNoiseVariances << .01, 100, .01;//100, 100;

    // This assumes that the normal vectors is (1,0,0) and the tangential (0,1,0) and (0,0,1)
    MatrixType fiducialCovMatrix = anisotropicNoiseVariances.asDiagonal();

    PointValuePairType funkyPointPair(middleFiducial,funkyTargetPoint);
    PointValueWithCovariancePairType pointValueWithCovariancePair(funkyPointPair, fiducialCovMatrix);
    PointValueWithCovarianceListType pointValueWithCovarianceList;
    pointValueWithCovarianceList.push_back(pointValueWithCovariancePair);

    //PointType upperPoint(420, 278, 0);
    //PointType lowerPoint(166, 266, 0);
    //PointValueWithCovariancePairType upperPair(PointValuePairType(upperPoint,upperPoint),MatrixType::Identity(3,3));
    //PointValueWithCovariancePairType lowerPair(PointValuePairType(lowerPoint,lowerPoint),MatrixType::Identity(3,3));
    //pointValueWithCovarianceList.push_back(upperPair);
    //pointValueWithCovarianceList.push_back(lowerPair);

    PosteriorModelBuilderType* anisotropicPosteriorModelBuilder = PosteriorModelBuilderType::Create();
    StatisticalModelType* anisotropicPosteriorModel
        = anisotropicPosteriorModelBuilder->BuildNewModelFromModel(fullModel, pointValueWithCovarianceList, false);

    vtkPolyData* posteriorMean = anisotropicPosteriorModel->DrawMean();

    double probability = fullModel->ComputeLogProbability(posteriorMean);

    //writePolyData(posteriorMean,"/local/tmp/hand-test.vtk");
    //fullModel->Save("/local/tmp/hand-model.h5");
    //std::cout << probability << std::endl;

    if(probability < -10) {
        testsOk = false;
        std::cout << "The probability within the original model of the posterior mean is too small:" << probability << std::endl;
        std::cout << "This means that the anisotropic noise model does not work." << std::endl;
    }


    VectorType coeffs = fullModel->ComputeCoefficientsForPointValuesWithCovariance(pointValueWithCovarianceList);
    VectorType coeffsFromPosteriorMean = fullModel->ComputeCoefficients(posteriorMean);
    VectorType difference = coeffs - coeffsFromPosteriorMean;
    double norm_difference = difference.norm();
    if(norm_difference > tolerance) {
        testsOk = false;
        std::cout << "The posterior mean computed by the StatisticalShapeModel class" << std::endl;
        std::cout << "and the one computed by the PosteriorModelBuilder are different." << std::endl;
        std::cout << "The norm of their difference is " << norm_difference << std::endl;
    }


    vtkPolyData* onePointPD = vtkPolyData::New();
    vtkPoints* onePoint = vtkPoints::New();
    onePoint->Allocate(1);
    onePoint->InsertNextPoint(0,0,0);
    onePointPD->SetPoints(onePoint);
    RepresenterType* onePointRepresenter = RepresenterType::Create(onePointPD);
    VectorType onePointMean(3);
    onePointMean << 0, 0, 0;
    VectorType onePointVar(3);
    onePointVar  << 1, 1, 1;
    MatrixType onePointPCABasis = MatrixType::Identity(3,3);

    StatisticalModelType* onePointModel = StatisticalModelType::Create(onePointRepresenter,
                                          onePointMean,
                                          onePointPCABasis,
                                          onePointVar,
                                          0.0);


    Eigen::Matrix3f rotMatrix;
    rotMatrix = Eigen::AngleAxisf(40,Eigen::Vector3f(0,0,1));

    VectorType covTestVector(3);
    covTestVector  <<  1,0,0;
    VectorType covTestVector1(3);
    covTestVector1 <<  0,2,0;
    VectorType covTestVector2(3);
    covTestVector2 <<  0,0,3;

    MatrixType testMatrix(3,3);
    testMatrix.col(0) = covTestVector;
    testMatrix.col(1) = covTestVector1;
    testMatrix.col(2) = covTestVector2;

    testMatrix = rotMatrix * testMatrix;

    VectorType testValuePoint(3);
    testValuePoint << 1,1,1;
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
    VectorType knownSolution(3);
    knownSolution << 0.5, 0.2, 0.1;

    if ((posteriorModelMean - knownSolution).norm() > 0.00001) {
        std::cout << "One point model test failed: Mean not correct." << std::endl;
        testsOk = false;
    }

    VectorType posteriorModelVariance = onePointPosteriorModel->GetPCAVarianceVector();
    VectorType knownVariance(3);
    knownVariance << 0.9, 0.8, 0.5;
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
    } else {
        std::cout << "Tests failed." << std::endl;
        return EXIT_FAILURE;
    }

}


