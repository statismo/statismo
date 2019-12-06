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


// This example shows how a partial shape can be reconstructed using a statistical shape model.
//
// WARNING: This example assumes that the closest point of the partial shape to the model mean is also its
// corresponding point. This is only true if the partial shape is close to the model mean. If this is not the
// case, a more sophisticated method for establishing correspondence needs to be used.
//
#include <iostream>

#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
#include <vtkPolyDataWriter.h>
#include <vtkVersion.h>

#include "statismo/core/DataManager.h"
#include "statismo/core/PosteriorModelBuilder.h"
#include "statismo/core/StatisticalModel.h"
#include "statismo/core/IO.h"

#include "statismo/VTK/vtkStandardMeshRepresenter.h"

#include <memory>

typedef statismo::VectorType                             VectorType;
typedef statismo::MatrixType                             MatrixType;
typedef statismo::vtkStandardMeshRepresenter             RepresenterType;
typedef statismo::StatisticalModel<vtkPolyData>          StatisticalModelType;
typedef statismo::PosteriorModelBuilder<vtkPolyData>     PosteriorModelBuilderType;
typedef StatisticalModelType::DomainType                 DomainType;
typedef DomainType::DomainPointsListType::const_iterator DomainPointsConstIterator;


vtkPolyData *
loadVTKPolyData(const std::string & filename)
{
  vtkPolyDataReader * reader = vtkPolyDataReader::New();
  reader->SetFileName(filename.c_str());
  reader->Update();
  vtkPolyData * pd = vtkPolyData::New();
  pd->DeepCopy(reader->GetOutput());
  reader->Delete();
  return pd;
}


/**
 * Computes the mahalanobis distance of the targetPt, to the model point with the given pointId.
 */
double
mahalanobisDistance(const StatisticalModelType * model, unsigned ptId, const statismo::vtkPoint & targetPt)
{
  statismo::MatrixType cov = model->GetCovarianceAtPoint(ptId, ptId);
  statismo::vtkPoint   meanPt = model->DrawMeanAtPoint(ptId);
  unsigned             pointDim = model->GetRepresenter()->GetDimensions();
  assert(pointDim <= 3);

  VectorType x = VectorType::Zero(pointDim);
  for (unsigned d = 0; d < pointDim; d++)
  {
    x(d) = targetPt[d] - meanPt[d];
  }
  return x.transpose() * cov.inverse() * x;
}


/**
 * Given the inputModel and a partialMesh, the program outputs the posterior model (a statistical model) and
 * its mean, which is the best reconstruction of the partial shape given the model.
 */
int
main(int argc, char ** argv)
{

  if (argc < 5)
  {
    std::cout << "Usage " << argv[0] << " inputModel  partialShapeMesh posteriorModel reconstructedShape" << std::endl;
    exit(-1);
  }


  std::string inputModelName(argv[1]);
  std::string partialShapeMeshName(argv[2]);
  std::string posteriorModelName(argv[3]);
  std::string reconstructedShapeName(argv[4]);

  try
  {


    vtkPolyData * partialShape = loadVTKPolyData(partialShapeMeshName);

    auto          representer = RepresenterType::SafeCreate();
    auto          inputModel = statismo::IO<vtkPolyData>::LoadStatisticalModel(representer.get(), inputModelName);
    vtkPolyData * refPd = const_cast<vtkPolyData *>(inputModel->GetRepresenter()->GetReference());


    StatisticalModelType::PointValueListType constraints;

    // for each point we get the closest point and check how far it is away (in mahalanobis distance).
    // If it is close, we add it as a constraint, otherwise we ignore the remaining points.
    const DomainType::DomainPointsListType & domainPoints = inputModel->GetDomain().GetDomainPoints();
    for (unsigned ptId = 0; ptId < domainPoints.size(); ptId++)
    {
      statismo::vtkPoint domainPoint = domainPoints[ptId];

      unsigned           closestPointId = ptId;
      statismo::vtkPoint closestPointOnPartialShape = partialShape->GetPoint(closestPointId);
      double             mhdist = mahalanobisDistance(inputModel.get(), ptId, closestPointOnPartialShape);
      if (mhdist < 5)
      {
        StatisticalModelType::PointValuePairType ptWithTargetPt(domainPoint, closestPointOnPartialShape);
        constraints.push_back(ptWithTargetPt);
      }
    }

    // build the new model. In addition to the input model and the constraints, we also specify
    // the inaccuracy of our value (variance of the error).

    auto posteriorModelBuilder = PosteriorModelBuilderType::SafeCreate();
    auto constraintModel = posteriorModelBuilder->BuildNewModelFromModel(inputModel.get(), constraints, 0.5);


    // The resulting model is a normal statistical model, from which we could for example sample examples.
    // Here we simply  save it to disk for later use.
    statismo::IO<vtkPolyData>::SaveStatisticalModel(constraintModel.get(), posteriorModelName);
    std::cout << "successfully saved the model to " << posteriorModelName << std::endl;

    // The mean of the constraint model is the optimal reconstruction
    vtkPolyData *       pmean = constraintModel->DrawMean();
    vtkPolyDataWriter * writer = vtkPolyDataWriter::New();
#if (VTK_MAJOR_VERSION == 5)
    writer->SetInput(pmean);
#else
    writer->SetInputData(pmean);
#endif
    writer->SetFileName(reconstructedShapeName.c_str());
    writer->Update();
  }
  catch (statismo::StatisticalModelException & e)
  {
    std::cout << "Exception occured while building the intenisity model" << std::endl;
    std::cout << e.what() << std::endl;
  }
}
