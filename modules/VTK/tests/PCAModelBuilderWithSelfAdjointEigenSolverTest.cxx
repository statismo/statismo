/*
 * This file is part of the statismo library.
 *
 * Author: Christoph Jud (christoph.jud@unibas.ch)
 *
 * Copyright (c) 2015 University of Basel
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

#include <Eigen/Geometry>

#include <vtkMath.h>
#include <vtkPoints.h>
#include <vtkPolyDataReader.h>
#include <vtkPolyDataWriter.h>
#include <vtkVersion.h>

#include "statismo/core/CommonTypes.h"
#include "statismo/core/DataManager.h"
#include "statismo/core/Domain.h"
#include "statismo/core/RandUtils.h"
#include "statismo/core/PCAModelBuilder.h"
#include "statismo/VTK/vtkStandardMeshRepresenter.h"
#include "vtkTestHelper.h"

#include <memory>
#include <ctime>
#include <vector>

using namespace statismo;
using namespace statismo::test;

/**
 * The test works as follows:
 *  - First a standard PCA model is built out of the sample hand-shapes of statismo
 *    It is ensured that the standard argument results in exactly the same model
 *    as if JacobiSVD is provided.
 *  - In the second test, a PCA model is built out of 5000 samples drawn from the
 *    previous built model. Here, the SelfAdjointEigenSolver is used, since there
 *    are more samples than variables.
 *
 * Arguments:
 *  - one can provide a number of points with which the samples should be subsampled (default is 100)
 *  - additionally, one can provide an output directory. If this is provided, the principal
 *    components of the model are sampled and stored in this directory. This is meant for visual inspection.
 */

int
PCAModelBuilderWithSelfAdjointEigenSolverTest(int argc, char ** argv)
{

  auto rg = rand::RandGen(0);

  if (argc < 2)
  {
    std::cout << "Usage: " << argv[0] << " datadir "
              << "[number_of_points=100] [output_dir]" << std::endl;
    exit(EXIT_FAILURE);
  }
  std::string datadir = std::string(argv[1]);

  bool testsOk = true;

  // number of points which are used for building the model
  // the number of points is reduced since the SelfAdjointEigenSolver operates
  // on the full covariance matrix. So
  unsigned num_points = 100;
  if (argc == 3)
    num_points = std::atoi(argv[2]);

  std::string output_dir = "";
  if (argc == 4)
    output_dir = argv[3];

  std::vector<std::string> filenames;
  filenames.push_back(datadir + "/hand_polydata/hand-0.vtk");
  filenames.push_back(datadir + "/hand_polydata/hand-1.vtk");
  filenames.push_back(datadir + "/hand_polydata/hand-2.vtk");
  filenames.push_back(datadir + "/hand_polydata/hand-3.vtk");
  filenames.push_back(datadir + "/hand_polydata/hand-4.vtk");
  filenames.push_back(datadir + "/hand_polydata/hand-5.vtk");
  filenames.push_back(datadir + "/hand_polydata/hand-6.vtk");
  filenames.push_back(datadir + "/hand_polydata/hand-7.vtk");
  filenames.push_back(datadir + "/hand_polydata/hand-8.vtk");
  filenames.push_back(datadir + "/hand_polydata/hand-9.vtk");
  filenames.push_back(datadir + "/hand_polydata/hand-10.vtk");
  filenames.push_back(datadir + "/hand_polydata/hand-11.vtk");
  filenames.push_back(datadir + "/hand_polydata/hand-12.vtk");
  filenames.push_back(datadir + "/hand_polydata/hand-13.vtk");
  filenames.push_back(datadir + "/hand_polydata/hand-14.vtk");
  filenames.push_back(datadir + "/hand_polydata/hand-15.vtk");
  filenames.push_back(datadir + "/hand_polydata/hand-16.vtk");


  typedef vtkStandardMeshRepresenter              RepresenterType;
  typedef statismo::BasicDataManager<vtkPolyData> DataManagerType;
  typedef vtkStandardMeshRepresenter::PointType   PointType;
  typedef vtkStandardMeshRepresenter::DomainType  DomainType;
  typedef DomainType::DomainPointsListType        DomainPointsListType;
  typedef statismo::StatisticalModel<vtkPolyData> StatisticalModelType;


  auto reference = LoadPolyData(filenames[0]);
  reference = ReducePoints(reference, num_points);
  auto representer = RepresenterType::SafeCreate(reference);

  auto dataManager = DataManagerType::SafeCreate(representer.get());

  std::vector<std::string>::const_iterator it = filenames.begin();
  for (; it != filenames.end(); it++)
  {
    auto testDataset = LoadPolyData((*it));
    testDataset = ReducePoints(testDataset, num_points);
    dataManager->AddDataset(testDataset, "dataset");
  }


  // ----------------------------------------------------------
  // First compute PCA model using standard JacobiSVD
  // ----------------------------------------------------------
  std::cout << "PCAModelBuilderWithSelfAdjointEigenSolverTest: \t"
            << "building PCA model with JacobiSVD... " << std::flush;
  std::clock_t                                   begin = std::clock();
  double                                         data_noise = 0;
  typedef statismo::PCAModelBuilder<vtkPolyData> PCAModelBuilderType;
  auto                                           pcaModelBuilder = PCAModelBuilderType::SafeCreate();

  // perform with standard argument
  auto       jacobiModel = pcaModelBuilder->BuildNewModel(dataManager->GetData(), data_noise, false);
  VectorType variance1 = jacobiModel->GetPCAVarianceVector();
  MatrixType pcbasis1 = jacobiModel->GetPCABasisMatrix();

  // perform with providing JacobiSVD
  jacobiModel =
    pcaModelBuilder->BuildNewModel(dataManager->GetData(), data_noise, false, PCAModelBuilderType::JacobiSVD);

  VectorType variance2 = jacobiModel->GetPCAVarianceVector();
  MatrixType pcbasis2 = jacobiModel->GetPCABasisMatrix();

  if (CompareVectors(variance1, variance2) == 0 && CompareMatrices(pcbasis1, pcbasis2) == 0)
  {
    std::clock_t end = std::clock();
    std::cout << " (" << double(end - begin) / CLOCKS_PER_SEC << " sec) \t\t[passed]" << std::endl;
  }
  else
  {
    std::cout << " \t[failed]" << std::endl;
    std::cout << "PCAModelBuilderWithSelfAdjointEigenSolverTest: \t\t\t"
              << "- something went wrong with the standard argument!" << std::endl;
    testsOk = false;
  }


  // ----------------------------------------------------------
  // Generate a lot of samples out of the JacobiSVD model
  // ----------------------------------------------------------
  auto dataManager2 = DataManagerType::SafeCreate(representer.get());
  dataManager->AddDataset(reference, "ref");
  for (unsigned i = 0; i < 5000; i++)
  {
    std::stringstream ss;
    ss << "sample" << i;
    dataManager2->AddDataset(jacobiModel->DrawSample(), ss.str().c_str());
  }


  // ----------------------------------------------------------
  // Compute PCA model using the SelfAdjointEigenSolver
  // ----------------------------------------------------------
  std::cout << "PCAModelBuilderWithSelfAdjointEigenSolverTest: \t"
            << "building PCA model with SelfAdjointEigenSolver... " << std::flush;
  begin = std::clock();
  auto saesModel = pcaModelBuilder->BuildNewModel(
    dataManager2->GetData(), data_noise, false, PCAModelBuilderType::SelfAdjointEigenSolver);
  std::clock_t end = std::clock();
  std::cout << " (" << double(end - begin) / CLOCKS_PER_SEC << " sec) \t[passed]" << std::endl;


  // ----------------------------------------------------------
  // Comparing the models
  // - compare each principal component
  // ----------------------------------------------------------
  std::cout << "PCAModelBuilderWithSelfAdjointEigenSolverTest: \t"
            << "comparing principal components... " << std::flush;
  begin = std::clock();
  double error = 0;
  for (unsigned i = 0;
       i < std::min(jacobiModel->GetNumberOfPrincipalComponents(), saesModel->GetNumberOfPrincipalComponents());
       i++)
  {
    VectorType coeff1 = VectorType::Zero(jacobiModel->GetNumberOfPrincipalComponents());
    coeff1[i] = 2;
    VectorType coeff2 = VectorType::Zero(saesModel->GetNumberOfPrincipalComponents());
    coeff2[i] = 2;

    // it might be, that the direction of the pc is in the opposite direction
    double equal_direction =
      CompareVectors(jacobiModel->DrawSampleVector(coeff1, false), saesModel->DrawSampleVector(coeff2, false));
    coeff2[i] = -2;
    double oppo_direction =
      CompareVectors(jacobiModel->DrawSampleVector(coeff1, false), saesModel->DrawSampleVector(coeff2, false));

    error += std::min(equal_direction, oppo_direction);

    if (output_dir.size() > 0)
    {
      std::stringstream ss1;
      ss1 << output_dir << "/jacobi-" << i << ".vtk";
      WritePolyData(jacobiModel->DrawSample(coeff1, false), ss1.str().c_str());

      std::stringstream ss2;
      ss2 << output_dir << "/saes-" << i << ".vtk";
      if (equal_direction < oppo_direction)
        coeff2[i] = 2;
      WritePolyData(saesModel->DrawSample(coeff2, false), ss2.str().c_str());
    }
  }

  double threshold = 150;
  if (error < threshold)
  {
    std::clock_t end = std::clock();
    std::cout << " (" << double(end - begin) / CLOCKS_PER_SEC << " sec) \t\t\t[passed]" << std::endl;
  }
  else
  {
    std::cout << " \t[failed]" << std::endl;
    std::cout << "PCAModelBuilderWithSelfAdjointEigenSolverTest: \t"
              << "- pc error(" << error << ") exceeds threshold (" << threshold << ")!" << std::endl;
    testsOk = false;
  }

  double var_error = 0;
  for (unsigned i = 0;
       i < std::min(jacobiModel->GetNumberOfPrincipalComponents(), saesModel->GetNumberOfPrincipalComponents());
       i++)
  {
    var_error += std::sqrt(std::pow(jacobiModel->GetPCAVarianceVector()[i] - saesModel->GetPCAVarianceVector()[i], 2));
  }

  std::cout << "PCAModelBuilderWithSelfAdjointEigenSolverTest: \t"
            << "comparing variances... " << std::flush;
  double var_threshold = 250;
  if (var_error < var_threshold)
  {
    std::clock_t end = std::clock();
    std::cout << " (" << double(end - begin) / CLOCKS_PER_SEC << " sec) \t\t\t\t[passed]" << std::endl;
  }
  else
  {
    std::cout << " \t[failed]" << std::endl;
    std::cout << "PCAModelBuilderWithSelfAdjointEigenSolverTest: \t"
              << "- variance error(" << var_error << ") exceeds threshold (" << var_threshold << ")!" << std::endl;
    testsOk = false;
  }


  if (output_dir.size() > 0)
  {
    std::cout << "PCAModelBuilderWithSelfAdjointEigenSolverTest: \t"
              << "The results can be visually inspected in the directory " << output_dir << std::endl;
  }

  if (testsOk == true)
  {
    std::cout << "PCAModelBuilderWithSelfAdjointEigenSolverTest: \t"
              << "Summary - tests passed." << std::endl;
    return EXIT_SUCCESS;
  }
  else
  {
    std::cout << "PCAModelBuilderWithSelfAdjointEigenSolverTest: \t"
              << "Summary - tests failed." << std::endl;
    return EXIT_FAILURE;
  }
}
