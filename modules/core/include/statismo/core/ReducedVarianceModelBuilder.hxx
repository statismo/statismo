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

#ifndef __STATIMO_CORE_REDUCED_VARIANCE_MODEL_BUILDER_HXX_
#define __STATIMO_CORE_REDUCED_VARIANCE_MODEL_BUILDER_HXX_

#include "statismo/core/ReducedVarianceModelBuilder.h"
#include "statismo/core/CommonTypes.h"
#include "statismo/core/Exceptions.h"

#include <Eigen/SVD>

#include <iostream>

namespace statismo
{

template <typename T>
UniquePtrType<typename ReducedVarianceModelBuilder<T>::StatisticalModelType>
ReducedVarianceModelBuilder<T>::BuildNewModelWithLeadingComponents(const StatisticalModelType * inputModel,
                                                                   unsigned numberOfPrincipalComponents) const

{
  auto reducedModel =
    StatisticalModelType::SafeCreate(inputModel->GetRepresenter(),
                                     inputModel->GetMeanVector(),
                                     inputModel->GetOrthonormalPCABasisMatrix().leftCols(numberOfPrincipalComponents),
                                     inputModel->GetPCAVarianceVector().topRows(numberOfPrincipalComponents),
                                     inputModel->GetNoiseVariance());

  // Write the parameters used to build the models into the builderInfo
  typename ModelInfo::BuilderInfoList builderInfoList = inputModel->GetModelInfo().GetBuilderInfoList();

  BuilderInfo::ParameterInfoList bi;
  bi.emplace_back("NumberOfPincipalComponents ", std::to_string(numberOfPrincipalComponents));

  BuilderInfo::DataInfoList di;
  builderInfoList.emplace_back("ReducedVarianceModelBuilder", di, bi);

  // If the scores matrix is not set, or if we have for some reasons not as many score entries as the number of
  // principal components, we simply work with what is there.
  unsigned numComponentsForScores =
    std::min(static_cast<unsigned>(inputModel->GetModelInfo().GetScoresMatrix().rows()), numberOfPrincipalComponents);

  reducedModel->SetModelInfo(
    ModelInfo(inputModel->GetModelInfo().GetScoresMatrix().topRows(numComponentsForScores), builderInfoList));
  return reducedModel;
}


template <typename T>
UniquePtrType<typename ReducedVarianceModelBuilder<T>::StatisticalModelType>
ReducedVarianceModelBuilder<T>::BuildNewModelWithVariance(const StatisticalModelType * inputModel,
                                                          double                       totalVariance) const
{

  VectorType pcaVariance = inputModel->GetPCAVarianceVector();
  double     modelVariance = pcaVariance.sum();

  // count the number of modes required for the model
  double   cumulatedVariance = 0;
  unsigned numComponentsToReachPrescribedVariance = 0;
  for (unsigned i = 0; i < pcaVariance.size(); i++)
  {
    cumulatedVariance += pcaVariance(i);
    numComponentsToReachPrescribedVariance++;
    if (cumulatedVariance / modelVariance >= totalVariance)
      break;
  }
  return BuildNewModelWithLeadingComponents(inputModel, numComponentsToReachPrescribedVariance);
}

template <typename T>
UniquePtrType<typename ReducedVarianceModelBuilder<T>::StatisticalModelType>
ReducedVarianceModelBuilder<T>::BuildNewModelFromModel(const StatisticalModelType * inputModel,
                                                       double                       totalVariance) const
{
  return BuildNewModelWithVariance(inputModel, totalVariance);
}

} // namespace statismo

#endif
