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


#include <Eigen/Geometry>

#include <vtkMath.h>
#include <vtkPoints.h>
#include <vtkPolyDataReader.h>
#include <vtkPolyDataWriter.h>
#include <vtkVersion.h>

#include "statismo/core/CommonTypes.h"
#include "statismo/core/DataManager.h"
#include "statismo/core/Domain.h"
#include "statismo/core/GenericRepresenterValidator.h"
#include "statismo/core/PCAModelBuilder.h"
#include "statismo/VTK/vtkStandardMeshRepresenter.h"
#include "vtkTestHelper.h"

#include <memory>
#include <ctime>
#include <vector>


using namespace statismo;
using namespace statismo::test;

typedef GenericRepresenterValidator<vtkStandardMeshRepresenter> RepresenterTestType;

int
PCAModelBuilderTest(int argc, char ** argv)
{
  if (argc < 2)
  {
    std::cout << "Usage: " << argv[0] << " datadir " << std::endl;
    exit(EXIT_FAILURE);
  }
  std::string datadir = std::string(argv[1]);

  bool testsOk = true;


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

  // ----------------------------------------------------------
  // First compute PCA model for case n > p
  // ----------------------------------------------------------
  unsigned num_points = 5; // use only 5 points as 5*3 coordinates < sample size (17)
  auto     reference = LoadPolyData(filenames[0]);
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
  VectorType baselineVariance1(10);
  baselineVariance1 << 460.601104736328125, 211.22674560546875, 107.32666015625, 71.84774017333984375,
    36.4659576416015625, 22.3681926727294921875, 11.6593990325927734375, 4.789171695709228515625,
    1.28080332279205322265625, 0.77941668033599853515625;

  std::cout << "PCAModelBuilderTest: \t"
            << "building PCA model with n > p... " << std::flush;
  std::clock_t                                   begin = std::clock();
  double                                         data_noise = 0;
  typedef statismo::PCAModelBuilder<vtkPolyData> PCAModelBuilderType;
  auto                                           pcaModelBuilder = PCAModelBuilderType::SafeCreate();
  // StatisticalModelType *                         PCAModel;

  // perform with standard argument
  auto       PCAModel = pcaModelBuilder->BuildNewModel(dataManager->GetData(), data_noise, false);
  VectorType variance1 = PCAModel->GetPCAVarianceVector();

  // max. acceptable difference between expected and calculated values in percent
  VectorType::Scalar maxPermittedDifference = 0.1; // 1‰

  if (CompareVectors(variance1, baselineVariance1) < maxPermittedDifference)
  {
    std::clock_t end = std::clock();
    std::cout << " (" << double(end - begin) / CLOCKS_PER_SEC << " sec) \t\t[passed]" << std::endl;
  }
  else
  {
    std::cout << " \t[failed]" << std::endl;
    std::cout << "PCAModelBuilder for sample size > variables: \t\t\t"
              << "-  computed variances are incorrect!" << std::endl;
    testsOk = false;
  }

  // ----------------------------------------------------------
  // Now compute PCA model for case p > n
  // ----------------------------------------------------------
  VectorType baselineVariance2(16);
  baselineVariance2 << 5175.92236328125, 3022.61181640625, 1786.9608154296875, 1131.9517822265625, 727.96649169921875,
    480.115386962890625, 365.292266845703125, 233.9134063720703125, 173.226318359375, 164.652557373046875,
    128.6950531005859375, 91.76165008544921875, 80.23679351806640625, 69.49117279052734375, 50.3206024169921875,
    42.5595245361328125;
  num_points = 100;

  auto reference2 = LoadPolyData(filenames[0]);
  reference2 = ReducePoints(reference2, num_points);
  auto representer2 = RepresenterType::SafeCreate(reference2);
  auto dataManager2 = DataManagerType::SafeCreate(representer2.get());

  for (it = filenames.begin(); it != filenames.end(); it++)
  {
    auto testDataset = LoadPolyData((*it));
    testDataset = ReducePoints(testDataset, 100);
    dataManager2->AddDataset(testDataset, "dataset");
  }
  std::cout << "PCAModelBuilderTest: \t"
            << "building PCA model with n < p... " << std::flush;
  begin = std::clock();

  typedef statismo::PCAModelBuilder<vtkPolyData> PCAModelBuilderType;
  auto                                           pcaModelBuilder2 = PCAModelBuilderType::SafeCreate();
  // StatisticalModelType *                         PCAModel2;

  // perform with standard argument
  auto       PCAModel2 = pcaModelBuilder2->BuildNewModel(dataManager2->GetData(), data_noise, false);
  VectorType variance2 = PCAModel2->GetPCAVarianceVector();

  if (CompareVectors(variance2, baselineVariance2) < maxPermittedDifference)
  {
    std::clock_t end = std::clock();
    std::cout << " (" << double(end - begin) / CLOCKS_PER_SEC << " sec) \t\t[passed]" << std::endl;
  }
  else
  {
    std::cout << " \t[failed]" << std::endl;
    std::cout << "PCAModelBuilder for sample size < variables: \t\t\t"
              << "- computed variances are incorrect!" << std::endl;
    testsOk = false;
  }

  if (testsOk == true)
  {
    std::cout << "PCAModelBuilderTest: \t"
              << "Summary - tests passed." << std::endl;
    return EXIT_SUCCESS;
  }
  else
  {
    std::cout << "PCAModelBuilderTest: \t"
              << "Summary - tests failed." << std::endl;
    return EXIT_FAILURE;
  }
}
