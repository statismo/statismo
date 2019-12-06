/*
 * This file is part of the statismo library.
 *
 * Author: Marcel Luethi (marcel.luethi@unibas.ch)
 *
 * Copyright (c) 2011 University of Basel
 * All rights reserved.
 *
 * Statismo is licensed under the BSD licence (3 clause) license
 */

#include <iostream>
#include <memory>

#include <vtkPolyDataReader.h>

#include "statismo/core/Kernels.h"
#include "statismo/core/KernelCombinators.h"
#include "statismo/core/LowRankGPModelBuilder.h"
#include "statismo/core/StatisticalModel.h"
#include "statismo/core/IO.h"
#include "statismo/VTK/vtkStandardMeshRepresenter.h"

using namespace statismo;


/*
 * We use a sum of gaussian kernels as our main model.
 */
class MultiscaleGaussianKernel : public MatrixValuedKernel<vtkPoint>
{
public:
  MultiscaleGaussianKernel(float baseWidth, float baseScale, unsigned nLevels)
    : m_baseWidth(baseWidth)
    , m_baseScale(baseScale)
    , m_nLevels(nLevels)
    , MatrixValuedKernel<vtkPoint>(3)
  {}

  inline MatrixType
  operator()(const vtkPoint & x, const vtkPoint & y) const
  {
    VectorType r(3);
    r << x[0] - y[0], x[1] - y[1], x[2] - y[2];

    const float minusRDotR = -r.dot(r);

    float kernelValue = 0.;
    for (unsigned l = 1; l <= m_nLevels; ++l)
    {
      const float scaleOnLevel = m_baseScale / static_cast<float>(l);
      const float widthOnLevel = m_baseWidth / static_cast<float>(l);
      kernelValue += scaleOnLevel * std::exp(minusRDotR / (widthOnLevel * widthOnLevel));
    }
    return statismo::MatrixType::Identity(3, 3) * kernelValue;
  }

  std::string
  GetKernelInfo() const
  {
    std::ostringstream os;
    os << "Multiscale GaussianKernel";
    return os.str();
  }

private:
  float    m_baseWidth;
  float    m_baseScale;
  unsigned m_nLevels;
};


// load a mesh
vtkPolyData *
loadVTKPolyData(const std::string & filename)
{
  vtkPolyDataReader * reader = vtkPolyDataReader::New();
  reader->SetFileName(filename.c_str());
  reader->Update();
  vtkPolyData * pd = vtkPolyData::New();
  pd->ShallowCopy(reader->GetOutput());
  reader->Delete();
  return pd;
}

// compute the center of mass for the given mesh
vtkPoint
centerOfMass(const vtkPolyData * _pd)
{

  // vtk is not const-correct, but we will not mutate pd here;
  vtkPolyData * pd = const_cast<vtkPolyData *>(_pd);

  vtkIdType numPoints = pd->GetNumberOfPoints();
  vtkPoint  centerOfMass(0.0, 0.0, 0.0);
  for (vtkIdType i = 0; i < numPoints; ++i)
  {
    double * ithPoint = pd->GetPoint(i);
    for (unsigned d = 0; d < 3; ++d)
    {
      centerOfMass[d] += ithPoint[d];
    }
  }
  double V = 1.0 / static_cast<double>(numPoints);
  for (unsigned d = 0; d < 3; ++d)
  {
    centerOfMass[d] *= V;
  }
  return centerOfMass;
}

// As an example of a tempering function, we use a function which is more smooth for points whose
// x-component is smaller than the x-component of the center of mass. To achieve a smooth transition between the areas,
// we use a sigmoid function. The variable a controls how fast the value of the tempering function changes from 0 to 1.
struct MyTemperingFunction : public TemperingFunction<vtkPoint>
{
  MyTemperingFunction(const vtkPoint & centerOfMass)
    : m_centerOfMass(centerOfMass)
  {}

  static const double a;
  double
  operator()(const vtkPoint & pt) const
  {
    double xDiffToCenter = m_centerOfMass[0] - pt[0];
    return (1.0 / (1.0 + std::exp(-xDiffToCenter * a)) + 1.0);
  }

private:
  vtkPoint m_centerOfMass;
};
const double MyTemperingFunction::a = 0.5;


//
// Computes a multi-scale gaussian model and uses spatial tempering to make the smoothness spatially varying.
//
// For mathematical details of this procedure, refer to the paper:
// T. Gerig, K. Shahim, M. Reyes, T. Vetter, M. Luethi, Spatially-varying registration using Gaussian Processes.
//
int
main(int argc, char ** argv)
{

  if (argc < 7)
  {
    std::cerr << "Usage " << argv[0]
              << " referenceMesh baseKernelWidth baseScale numLevels numberOfComponents outputmodelName" << std::endl;
    return EXIT_FAILURE;
  }
  std::string refFilename(argv[1]);
  double      baseKernelWidth = std::atof(argv[2]);
  double      baseScale = std::atof(argv[3]);
  unsigned    numLevels = std::atoi(argv[4]);
  int         numberOfComponents = std::atoi(argv[5]);
  std::string outputModelFilename(argv[6]);


  // All the statismo classes have to be parameterized with the RepresenterType.

  typedef vtkStandardMeshRepresenter         RepresenterType;
  typedef LowRankGPModelBuilder<vtkPolyData> ModelBuilderType;
  typedef StatisticalModel<vtkPolyData>      StatisticalModelType;
  typedef MultiscaleGaussianKernel           GaussianKernelType;
  typedef MatrixValuedKernel<vtkPoint>       MatrixValuedKernelType;

  try
  {

    vtkPolyData *                referenceMesh = loadVTKPolyData(refFilename);
    vtkStandardMeshRepresenter * representer = vtkStandardMeshRepresenter::Create(referenceMesh);

    MultiscaleGaussianKernel gk = MultiscaleGaussianKernel(baseKernelWidth, baseScale, numLevels);

    MyTemperingFunction temperingFun(centerOfMass(referenceMesh));

    SpatiallyVaryingKernel<RepresenterType::DatasetType> temperedKernel(
      representer, gk, temperingFun, numberOfComponents, numberOfComponents * 2, true);

    // We create a new model using the combined kernel. The new model will be more flexible than the original
    // statistical model.
    auto modelBuilder = ModelBuilderType::SafeCreate(representer);

    auto newModel = modelBuilder->BuildNewModel(referenceMesh, temperedKernel, numberOfComponents);

    // Once we have built the model, we can save it to disk.
    statismo::IO<vtkPolyData>::SaveStatisticalModel(newModel.get(), outputModelFilename);
    std::cout << "Successfully saved shape model as " << outputModelFilename << std::endl;

    referenceMesh->Delete();
    representer->Delete();
    modelBuilder->Delete();
    newModel->Delete();
  }
  catch (StatisticalModelException & e)
  {
    std::cerr << "Exception occured while building the shape model" << std::endl;
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
