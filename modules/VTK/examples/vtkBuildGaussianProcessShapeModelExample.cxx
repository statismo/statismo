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

#include <vtkPolyDataReader.h>

#include "statismo/core/Kernels.h"
#include "statismo/core/KernelCombinators.h"
#include "statismo/core/LowRankGPModelBuilder.h"
#include "statismo/core/StatisticalModel.h"
#include "statismo/core/IO.h"

#include "statismo/VTK/vtkStandardMeshRepresenter.h"

#include <memory>

using namespace statismo;


/**
 * A scalar valued gaussian kernel.
 */
class GaussianKernel : public ScalarValuedKernel<vtkPoint>
{
public:
  GaussianKernel(double sigma)
    : m_sigma(sigma)
    , m_sigma2(sigma * sigma)
  {}

  inline double
  operator()(const vtkPoint & x, const vtkPoint & y) const
  {
    VectorType r(3);
    r << x[0] - y[0], x[1] - y[1], x[2] - y[2];
    return exp(-r.dot(r) / m_sigma2);
  }

  std::string
  GetKernelInfo() const
  {
    std::ostringstream os;
    os << "GaussianKernel(" << m_sigma << ")";
    return os.str();
  }

private:
  double m_sigma;
  double m_sigma2;
};


vtkPolyData *
loadVTKPolyData(const std::string & filename)
{
  vtkPolyDataReader * reader = vtkPolyDataReader::New();
  reader->SetFileName(filename.c_str());
  reader->Update();
  vtkPolyData * pd = vtkPolyData::New();
  pd->ShallowCopy(reader->GetOutput());
  return pd;
}


//
// This example illustrates, how the flexibility of a statistical (shape) model can be extended by combining its
// covariance function with a Gaussian Kernel function.
//
int
main(int argc, char ** argv)
{

  if (argc < 5)
  {
    std::cout << "Usage " << argv[0] << " model gaussianKernelWidth numberOfComponents, outputmodelName" << std::endl;
    exit(-1);
  }
  std::string modelFilename(argv[1]);
  double      gaussianKernelSigma = std::atof(argv[2]);
  int         numberOfComponents = std::atoi(argv[3]);
  std::string outputModelFilename(argv[4]);


  // All the statismo classes have to be parameterized with the RepresenterType.

  typedef vtkStandardMeshRepresenter         RepresenterType;
  typedef LowRankGPModelBuilder<vtkPolyData> ModelBuilderType;
  typedef StatisticalModel<vtkPolyData>      StatisticalModelType;
  typedef GaussianKernel                     GaussianKernelType;
  typedef MatrixValuedKernel<vtkPoint>       MatrixValuedKernelType;

  try
  {

    // we load an existing statistical model and create a StatisticalModelKernel from it. The statisticlModelKernel
    // takes the covariance (matrix) of the model and defines a kernel function from it.
    vtkStandardMeshRepresenter *   representer = vtkStandardMeshRepresenter::Create();
    auto                           model = statismo::IO<vtkPolyData>::LoadStatisticalModel(representer, modelFilename);
    const MatrixValuedKernelType & statModelKernel = StatisticalModelKernel<vtkPolyData>(model.get());

    // Create a (scalar valued) gaussian kernel. This kernel is then made matrix-valued. We use a
    // UncorrelatedMatrixValuedKernel, which assumes that each output component is independent.

    const GaussianKernel           gk = GaussianKernel(gaussianKernelSigma);
    const MatrixValuedKernelType & mvGk =
      UncorrelatedMatrixValuedKernel<vtkPoint>(&gk, model->GetRepresenter()->GetDimensions());

    // We scale the kernel (and hence the resulting deformations) of the Gaussian kernel by  a factor of 100, in order
    // to achieve a visible effect.
    const MatrixValuedKernelType & scaledGk = ScaledKernel<vtkPoint>(&mvGk, 100.0);

    // The model kernel and the Gaussian kernel are combined to a new kernel.
    const MatrixValuedKernelType & combinedModelAndGaussKernel = SumKernel<vtkPoint>(&statModelKernel, &scaledGk);

    // We create a new model using the combined kernel. The new model will be more flexible than the original
    // statistical model.
    auto modelBuilder = ModelBuilderType::SafeCreate(model->GetRepresenter());
    auto combinedModel =
      modelBuilder->BuildNewModel(model->DrawMean(), combinedModelAndGaussKernel, numberOfComponents);

    // Once we have built the model, we can save it to disk.
    statismo::IO<vtkPolyData>::SaveStatisticalModel(combinedModel.get(), outputModelFilename);
    std::cout << "Successfully saved shape model as " << outputModelFilename << std::endl;
  }
  catch (StatisticalModelException & e)
  {
    std::cout << "Exception occured while building the shape model" << std::endl;
    std::cout << e.what() << std::endl;
  }
}
