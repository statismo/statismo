/*
 * ConditionalBuilder.hxx
 *
 * Created by Remi Blanc, Marcel Luethi
 *
 * Copyright (c) 2011 ETH Zurich
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

#ifndef __STATIMO_CORE_CONDITIONAL_MODEL_BUILDER_HXX_
#define __STATIMO_CORE_CONDITIONAL_MODEL_BUILDER_HXX_

#include "statismo/core/ConditionalModelBuilder.h"
#include "statismo/core/Exceptions.h"
#include "statismo/core/PCAModelBuilder.h"

#include <Eigen/SVD>

#include <iostream>


namespace statismo
{

template <typename T>
unsigned
ConditionalModelBuilder<T>::PrepareData(const DataItemListType &            sampleDataList,
                                        const SurrogateTypeInfoType &       surrogateTypesInfo,
                                        const CondVariableValueVectorType & conditioningInfo,
                                        DataItemListType &                  acceptedSamples,
                                        MatrixType &                        surrogateMatrix,
                                        VectorType &                        conditions) const
{
  assert(conditioningInfo.size() == surrogateTypesInfo.types.size());

  // 1- identify the continuous and categorical variables, which are used for conditioning and which are not
  std::vector<unsigned> indicesContinuousSurrogatesInUse;
  std::vector<unsigned> indicesCategoricalSurrogatesInUse;
  for (unsigned i = 0; i < conditioningInfo.size(); i++)
  {
    if (conditioningInfo[i].first)
    {
      // only variables that are used for conditioning are of interest here
      if (surrogateTypesInfo.types[i] == DataItemWithSurrogatesType::SurrogateType::Continuous)
      {
        indicesContinuousSurrogatesInUse.push_back(i);
      }
      else
      {
        indicesCategoricalSurrogatesInUse.push_back(i);
      }
    }
  }

  conditions.resize(indicesContinuousSurrogatesInUse.size());
  for (unsigned i = 0; i < indicesContinuousSurrogatesInUse.size(); i++)
  {
    conditions(i) = conditioningInfo[indicesContinuousSurrogatesInUse[i]].second;
  }

  surrogateMatrix.resize(indicesContinuousSurrogatesInUse.size(),
                         sampleDataList.size()); // number of variables is now known: nbContinuousSurrogatesInUse ; the
                                                 // number of samples is yet unknown...

  // now, browse all samples to select the ones which fall into the requested categories
  for (const auto & item : sampleDataList)
  {
    const auto * sampleData = dynamic_cast<const DataItemWithSurrogatesType *>(item.get());
    if (!sampleData)
    {
      // this is a normal sample without surrogate information.
      // we simply discard it
      std::cout << "WARNING: ConditionalModelBuilder, sample data " << item->GetDatasetURI()
                << " has no surrogate data associated, and is ignored" << std::endl;
      continue;
    }

    auto surrogateData = sampleData->GetSurrogateVector();
    if (std::all_of(
          std::cbegin(indicesCategoricalSurrogatesInUse), std::cend(indicesCategoricalSurrogatesInUse), [&](auto i) {
            return conditioningInfo[indicesCategoricalSurrogatesInUse[i]].second ==
                   surrogateData[indicesCategoricalSurrogatesInUse[i]];
          }))
    {
      acceptedSamples.push_back(item);
      // and fill in the matrix of continuous variables
      for (unsigned j = 0; j < indicesContinuousSurrogatesInUse.size(); j++)
      {
        surrogateMatrix(j, acceptedSamples.size() - 1) = surrogateData[indicesContinuousSurrogatesInUse[j]];
      }
    }
  }
  // resize the matrix of surrogate data to the effective number of accepted samples
  surrogateMatrix.conservativeResize(Eigen::NoChange_t(), acceptedSamples.size());

  return acceptedSamples.size();
}

template <typename T>
UniquePtrType<typename ConditionalModelBuilder<T>::StatisticalModelType>
ConditionalModelBuilder<T>::BuildNewModel(const DataItemListType &            sampleDataList,
                                          const SurrogateTypeInfoType &       surrogateTypesInfo,
                                          const CondVariableValueVectorType & conditioningInfo,
                                          float                               noiseVariance,
                                          double                              modelVarianceRetained) const
{
  if (conditioningInfo.size() != surrogateTypesInfo.types.size())
  {
    throw StatisticalModelException("mismatch between conditioning info size and surrogates info size",
                                    Status::BAD_INPUT_ERROR);
  }

  DataItemListType acceptedSamples;
  MatrixType       X;
  VectorType       x0;
  auto             nSamples = PrepareData(sampleDataList, surrogateTypesInfo, conditioningInfo, acceptedSamples, X, x0);
  assert(nSamples == acceptedSamples.size());

  unsigned nCondVariables = X.rows();

  // build a normal PCA model
  using PCAModelBuilderType = PCAModelBuilder<T>;
  auto modelBuilder = PCAModelBuilderType::SafeCreate();
  auto pcaModel = modelBuilder->BuildNewModel(acceptedSamples, noiseVariance);

  unsigned nPCAComponents = pcaModel->GetNumberOfPrincipalComponents();

  if (X.cols() == 0 || X.rows() == 0)
  {
    return pcaModel;
  }
  else
  {
    // the scores in the pca model correspond to the parameters of each sample in the model.
    MatrixType B = pcaModel->GetModelInfo().GetScoresMatrix().transpose();
    assert(B.rows() == nSamples);
    assert(B.cols() == nPCAComponents);

    // A is the joint data matrix B, X, where X contains the conditional information for each sample
    // Thus the i-th row of A contains the PCA parameters b of the i-th sample,
    // together with the conditional information for each sample
    MatrixType A(nSamples, nPCAComponents + nCondVariables);
    A << (B, X.transpose());

    // Compute the mean and the covariance of the joint data matrix
    auto mu = A.colwise().mean().transpose(); // colwise returns a row vector
    assert(mu.rows() == nPCAComponents + nCondVariables);

    auto A0 = A.rowwise() - mu.transpose();
    auto cov = 1.0 / (nSamples - 1) * A0.transpose() * A0;

    assert(cov.rows() == cov.cols());
    assert(cov.rows() == (nPCAComponents + nCondVariables));

    // extract the submatrices involving the conditionals x
    // note that since the matrix is symmetric, Sbx = Sxb.transpose(), hence we only store one
    auto Sbx = cov.topRightCorner(nPCAComponents, nCondVariables);
    auto Sxx = cov.bottomRightCorner(nCondVariables, nCondVariables);
    auto Sbb = cov.topLeftCorner(nPCAComponents, nPCAComponents);

    // compute the conditional mean
    auto condMean = mu.topRows(nPCAComponents) + Sbx * Sxx.inverse() * (x0 - mu.bottomRows(nCondVariables));

    // compute the conditional covariance
    MatrixType condCov = Sbb - Sbx * Sxx.inverse() * Sbx.transpose();

    // get the sample mean corresponding the conditional given mean of the parameter vectors
    auto condMeanSample = pcaModel->GetRepresenter()->SampleToSampleVector(pcaModel->DrawSample(condMean));

    // so far all the computation have been done in parameter (latent) space. Go back to sample space.
    // (see PartiallyFixedModelBuilder for a detailed documentation)
    // TODO: we should factor this out into the base class, as it is the same code as it is used in
    // the partially fixed model builder
    const VectorType &        pcaVariance = pcaModel->GetPCAVarianceVector();
    VectorTypeDoublePrecision pcaSdev = pcaVariance.cast<double>().array().sqrt();

    using SVDType = Eigen::JacobiSVD<MatrixTypeDoublePrecision>;
    auto    innerMatrix = pcaSdev.asDiagonal() * condCov.cast<double>() * pcaSdev.asDiagonal();
    SVDType svd(innerMatrix, Eigen::ComputeThinU);
    auto    singularValues = svd.singularValues().cast<ScalarType>();

    // keep only the necessary number of modes, wrt modelVarianceRetained...
    double totalRemainingVariance = singularValues.sum(); //
    // and count the number of modes required for the model
    double   cumulatedVariance = singularValues(0);
    unsigned numComponentsToReachPrescribedVariance{ 1 };
    while ((cumulatedVariance / totalRemainingVariance) < modelVarianceRetained)
    {
      numComponentsToReachPrescribedVariance++;
      if (numComponentsToReachPrescribedVariance == singularValues.size())
      {
        break;
      }
      cumulatedVariance += singularValues(numComponentsToReachPrescribedVariance - 1);
    }

    unsigned numComponentsToKeep = std::min<unsigned>(numComponentsToReachPrescribedVariance, singularValues.size());

    auto newPCAVariance = singularValues.topRows(numComponentsToKeep);
    auto newPCABasisMatrix =
      (pcaModel->GetOrthonormalPCABasisMatrix() * svd.matrixU().cast<ScalarType>()).leftCols(numComponentsToKeep);

    auto model = StatisticalModelType::SafeCreate(
      pcaModel->GetRepresenter(), condMeanSample, newPCABasisMatrix, newPCAVariance, noiseVariance);

    // add builder info and data info to the info list
    MatrixType                     scores(0, 0);
    BuilderInfo::ParameterInfoList bi;
    bi.emplace_back("NoiseVariance ", std::to_string(noiseVariance));

    // generate a matrix ; first column = boolean (yes/no, this variable is used) ; second: conditioning value.
    MatrixType conditioningInfoMatrix(conditioningInfo.size(), 2);
    for (unsigned i = 0; i < conditioningInfo.size(); i++)
    {
      conditioningInfoMatrix(i, 0) = conditioningInfo[i].first;
      conditioningInfoMatrix(i, 1) = conditioningInfo[i].second;
    }
    bi.emplace_back("ConditioningInfo ", std::to_string(conditioningInfoMatrix));

    typename BuilderInfo::DataInfoList di;
    for (const auto & item : sampleDataList)
    {
      const auto *       sampleData = dynamic_cast<const DataItemWithSurrogatesType *>(item.get());
      std::ostringstream os;
      os << "URI_" << (di.size() / 2);
      di.emplace_back(os.str().c_str(), sampleData->GetDatasetURI());

      os << "_surrogates";
      di.emplace_back(os.str().c_str(), sampleData->GetSurrogateFilename());
    }

    di.emplace_back("surrogates_types", surrogateTypesInfo.typeFilename);

    ModelInfo::BuilderInfoList biList;
    biList.emplace_back("ConditionalModelBuilder", di, bi);
    model->SetModelInfo(ModelInfo{ scores, biList });

    return model;
  }
}

} // namespace statismo

#endif
