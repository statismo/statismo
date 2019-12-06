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
#include <iostream>

#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
#include <vtkPolyDataWriter.h>
#include <vtkVersion.h>

#include "statismo/core/StatisticalModel.h"
#include "statismo/core/IO.h"
#include "statismo/VTK/vtkStandardMeshRepresenter.h"

#include <memory>

using namespace statismo;

void
saveSample(const vtkPolyData * pd, const std::string & resdir, const std::string & basename)
{
  std::string filename = resdir + std::string("/") + basename;

  vtkPolyDataWriter * w = vtkPolyDataWriter::New();
#if (VTK_MAJOR_VERSION == 5)
  w->SetInput(const_cast<vtkPolyData *>(pd));
#else
  w->SetInputData(const_cast<vtkPolyData *>(pd));
#endif
  w->SetFileName(filename.c_str());
  w->Update();
}

// illustrates how to load a shape model and the basic sampling functinality
int
main(int argc, char ** argv)
{

  if (argc < 3)
  {
    std::cout << "Usage " << argv[0] << " modelname resultdir" << std::endl;
    exit(-1);
  }
  std::string modelname(argv[1]);
  std::string resultdir(argv[2]);


  // All the statismo classes have to be parameterized with the RepresenterType.
  // For building a shape model with vtk, we use the vtkPolyDataRepresenter.
  typedef vtkStandardMeshRepresenter    RepresenterType;
  typedef StatisticalModel<vtkPolyData> StatisticalModelType;

  try
  {

    // To load a model, we call the static Load method, which returns (a pointer to) a
    // new StatisticalModel object
    RepresenterType *                   representer = RepresenterType::Create();
    UniquePtrType<StatisticalModelType> model(statismo::IO<vtkPolyData>::LoadStatisticalModel(representer, modelname));
    std::cout << "loaded model with " << model->GetNumberOfPrincipalComponents() << " Principal Components"
              << std::endl;


    // get the model mean
    vtkPolyData * mean = model->DrawMean();
    saveSample(mean, resultdir, "mean.vtk");

    // draw a random sample
    vtkPolyData * randomSample = model->DrawSample();
    saveSample(randomSample, resultdir, "randomsample.vtk");

    // draw a sample with known pca coefficients (3 stddev in direction of the 1st PC)
    VectorType coefficients = VectorType::Zero(model->GetNumberOfPrincipalComponents());
    coefficients(0) = 3;
    vtkPolyData * samplePC1 = model->DrawSample(coefficients);
    saveSample(samplePC1, resultdir, "samplePC1.vtk");


    // The vtkPolyDataRepresenter returns naked pointers to vtk objcts. Therefore we have to delete all the samples
    mean->Delete();
    randomSample->Delete();
    samplePC1->Delete();

    std::cout << "saved samples to " << resultdir << std::endl;
  }
  catch (StatisticalModelException & e)
  {
    std::cout << "Exception occured while building the shape model" << std::endl;
    std::cout << e.what() << std::endl;
  }
}
