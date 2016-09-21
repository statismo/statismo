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
#include <ctime>
#include <vector>

#include <Eigen/Geometry>

#include <vtkMath.h>
#include <vtkPoints.h>
#include <vtkPolyDataReader.h>
#include <vtkPolyDataWriter.h>
#include <vtkVersion.h>

#include "CommonTypes.h"
#include "DataManager.h"
#include "Domain.h"
#include "genericRepresenterTest.hxx"
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

double CompareVectors(const VectorType& v1, const VectorType& v2) {
  return (v1-v2).array().abs().maxCoeff();
}

vtkPolyData* ReducePoints(vtkPolyData* poly, unsigned num_points) {
  vtkPoints* points = vtkPoints::New();
  unsigned step = unsigned(std::ceil(double(poly->GetPoints()->GetNumberOfPoints())/double(num_points)));
  for(unsigned i=0; i< num_points; i++) {
    points->InsertNextPoint(poly->GetPoints()->GetPoint(i));
  }
  vtkPolyData* res = vtkPolyData::New();
  res->SetPoints(points);
  return res;
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " datadir " << std::endl;
    exit(EXIT_FAILURE);
  }
  std::string datadir = std::string(argv[1]);

  bool testsOk = true;

 
  std::vector<std::string> filenames;
  filenames.push_back(datadir+"/hand_polydata/hand-0.vtk");
  filenames.push_back(datadir+"/hand_polydata/hand-1.vtk");
  filenames.push_back(datadir+"/hand_polydata/hand-2.vtk");
  filenames.push_back(datadir+"/hand_polydata/hand-3.vtk");
  filenames.push_back(datadir+"/hand_polydata/hand-4.vtk");
  filenames.push_back(datadir+"/hand_polydata/hand-5.vtk");
  filenames.push_back(datadir+"/hand_polydata/hand-6.vtk");
  filenames.push_back(datadir+"/hand_polydata/hand-7.vtk");
  filenames.push_back(datadir+"/hand_polydata/hand-8.vtk");
  filenames.push_back(datadir+"/hand_polydata/hand-9.vtk");
  filenames.push_back(datadir+"/hand_polydata/hand-10.vtk");
  filenames.push_back(datadir+"/hand_polydata/hand-11.vtk");
  filenames.push_back(datadir+"/hand_polydata/hand-12.vtk");
  filenames.push_back(datadir+"/hand_polydata/hand-13.vtk");
  filenames.push_back(datadir+"/hand_polydata/hand-14.vtk");
  filenames.push_back(datadir+"/hand_polydata/hand-15.vtk");
  filenames.push_back(datadir+"/hand_polydata/hand-16.vtk");

   
  typedef vtkStandardMeshRepresenter RepresenterType;
  typedef statismo::DataManager<vtkPolyData> DataManagerType;
  typedef vtkStandardMeshRepresenter::PointType PointType;
  typedef vtkStandardMeshRepresenter::DomainType DomainType;
  typedef DomainType::DomainPointsListType DomainPointsListType;
  typedef statismo::StatisticalModel<vtkPolyData> StatisticalModelType;

  // ----------------------------------------------------------
  // First compute PCA model for case n > p
  // ----------------------------------------------------------
  unsigned num_points = 5; //use only 5 points as 5*3 coordinates < sample size (17)
  vtkPolyData* reference = loadPolyData(filenames[0]);
  reference = ReducePoints(reference, num_points);
  RepresenterType* representer = RepresenterType::Create(reference);

  boost::scoped_ptr<DataManagerType> dataManager(DataManagerType::Create(representer));

  std::vector<std::string>::const_iterator it = filenames.begin();
  for(; it!=filenames.end(); it++) {
    vtkPolyData* testDataset = loadPolyData((*it));
    testDataset = ReducePoints(testDataset, num_points);
    dataManager->AddDataset(testDataset, "dataset");
  }
  VectorType baselineVariance1(10);
  baselineVariance1 << 1129.2266845703125, 269.25128173828125, 1.95318043231964111328125, 0.8879330158233642578125, 0.04632849991321563720703125, 0.01352225802838802337646484375, 0.0008153090602718293666839599609375, 0.00033547866041772067546844482421875, 5.807749766972847282886505126953125e-05, 1.56848909682594239711761474609375e-05;
  
  std::cout << "PCAModelBuilderTest: \t" << "building PCA model with n > p... " << std::flush;
  std::clock_t begin = std::clock();
  double data_noise = 0;
  typedef statismo::PCAModelBuilder<vtkPolyData> PCAModelBuilderType;
  PCAModelBuilderType* pcaModelBuilder = PCAModelBuilderType::Create();
  StatisticalModelType* PCAModel;
    
  // perform with standard argument
  PCAModel = pcaModelBuilder->BuildNewModel(dataManager->GetData(),data_noise,false);
  VectorType variance1 = PCAModel->GetPCAVarianceVector();
    
  if(CompareVectors(variance1, baselineVariance1) < 1e-8) {
    std::clock_t end = std::clock();
    std::cout << " (" << double(end - begin) / CLOCKS_PER_SEC << " sec) \t\t[passed]" << std::endl;
  } else {
    std::cout << " \t[failed]" << std::endl;
    std::cout << "PCAModelBuilder for sample size > variables: \t\t\t" << "-  computed variances are incorrect!" << std::endl;
    testsOk = false;
  }
  
  // ----------------------------------------------------------
  // Now compute PCA model for case p > n
  // ----------------------------------------------------------
  VectorType baselineVariance2(16);
  baselineVariance2 << 16644.25, 2851.044921875, 789.446044921875, 498.49322509765625, 296.288818359375, 119.069671630859375, 48.84352874755859375, 39.074352264404296875, 19.6847972869873046875, 16.53295135498046875, 12.06073093414306640625, 9.15244388580322265625, 7.496630191802978515625, 4.588232517242431640625, 3.5666046142578125, 2.4388735294342041015625;
  num_points = 100;

  vtkPolyData* reference2 = loadPolyData(filenames[0]);
  reference2 = ReducePoints(reference2, num_points);
  RepresenterType* representer2 = RepresenterType::Create(reference2);
  boost::scoped_ptr<DataManagerType> dataManager2(DataManagerType::Create(representer2));
  
  for(it = filenames.begin(); it!=filenames.end(); it++) {
    vtkPolyData* testDataset = loadPolyData((*it));
    testDataset = ReducePoints(testDataset, 100);
    dataManager2->AddDataset(testDataset, "dataset");
  }
  std::cout << "PCAModelBuilderTest: \t" << "building PCA model with n < p... " << std::flush;
  begin = std::clock();
  
  typedef statismo::PCAModelBuilder<vtkPolyData> PCAModelBuilderType;
  PCAModelBuilderType* pcaModelBuilder2 = PCAModelBuilderType::Create();
  StatisticalModelType* PCAModel2;
    
  // perform with standard argument
  PCAModel2 = pcaModelBuilder2->BuildNewModel(dataManager2->GetData(),data_noise,false);
  VectorType variance2 = PCAModel2->GetPCAVarianceVector();

  if(CompareVectors(variance2, baselineVariance2) < 1e-8) {
    std::clock_t end = std::clock();
    std::cout << " (" << double(end - begin) / CLOCKS_PER_SEC << " sec) \t\t[passed]" << std::endl;
  } else {
    std::cout << " \t[failed]" << std::endl;
    std::cout << "PCAModelBuilder for sample size < variables: \t\t\t" << "- computed variances are incorrect!" << std::endl;
    testsOk = false;
  }
    
  if (testsOk == true) {
    std::cout << "PCAModelBuilderTest: \t" << "Summary - tests passed." << std::endl;
    return EXIT_SUCCESS;
  } else {
    std::cout << "PCAModelBuilderTest: \t" << "Summary - tests failed." << std::endl;
    return EXIT_FAILURE;
  }
}

