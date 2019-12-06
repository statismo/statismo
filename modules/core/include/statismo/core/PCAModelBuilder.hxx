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

#ifndef __STATIMO_CORE_PCA_MODEL_BUILDER_HXX_
#define __STATIMO_CORE_PCA_MODEL_BUILDER_HXX_

#include "statismo/core/PCAModelBuilder.h"
#include "statismo/core/CommonTypes.h"
#include "statismo/core/Exceptions.h"

#include <Eigen/SVD>
#include <Eigen/Eigenvalues>

#include <iostream>

namespace statismo
{

template <typename T>
UniquePtrType<typename PCAModelBuilder<T>::StatisticalModelType>
PCAModelBuilder<T>::BuildNewModel(const DataItemListType & sampleDataList,
                                  double                   noiseVariance,
                                  bool                     computeScores,
                                  EigenValueMethod         method) const
{

  auto n = sampleDataList.size();
  if (n <= 0)
  {
    throw StatisticalModelException("Provided empty sample set. Cannot build the sample matrix",
                                    Status::BAD_INPUT_ERROR);
  }

  unsigned     p = sampleDataList.front()->GetSampleVector().rows();
  const auto * representer = sampleDataList.front()->GetRepresenter();

  // Compute the mean vector mu
  VectorType mu = VectorType::Zero(p);

  for (const auto & item : sampleDataList)
  {
    assert(item->GetSampleVector().rows() == p);   // all samples must have same number of rows
    assert(item->GetRepresenter() == representer); // all samples have the same representer
    mu += item->GetSampleVector();
  }

  mu /= n;

  // Build the mean free sample matrix X0
  MatrixType X0(n, p);
  unsigned   i{ 0 };
  for (const auto & item : sampleDataList)
  {
    X0.row(i++) = item->GetSampleVector() - mu;
  }

  // build the model
  auto model = BuildNewModelInternal(representer, X0, mu, noiseVariance, method);

  // compute the scores if requested
  MatrixType scores;
  if (computeScores)
  {
    scores = this->ComputeScores(sampleDataList, model.get());
  }

  typename BuilderInfo::ParameterInfoList bi;
  bi.emplace_back(BuilderInfo::KeyValuePair("NoiseVariance ", std::to_string(noiseVariance)));

  typename BuilderInfo::DataInfoList dataInfo;
  i = 0;
  for (const auto & item : sampleDataList)
  {
    std::ostringstream os;
    os << "URI_" << i++;
    dataInfo.emplace_back(os.str().c_str(), item->GetDatasetURI());
  }

  // finally add meta data to the model info
  ModelInfo::BuilderInfoList biList;
  biList.emplace_back("PCAModelBuilder", dataInfo, bi);

  model->SetModelInfo(ModelInfo{ scores, biList });

  return model;
}


template <typename T>
UniquePtrType<typename PCAModelBuilder<T>::StatisticalModelType>
PCAModelBuilder<T>::BuildNewModelInternal(const Representer<T> * representer,
                                          const MatrixType &     X0,
                                          const VectorType &     mu,
                                          double                 noiseVariance,
                                          EigenValueMethod       method) const
{

  unsigned n = X0.rows();
  unsigned p = X0.cols();

  switch (method)
  {
    case JacobiSVD:

      using SVDType = Eigen::JacobiSVD<MatrixType>;
      using SVDDoublePrecisionType = Eigen::JacobiSVD<MatrixTypeDoublePrecision>;

      // We destinguish the case where we have more variables than samples and
      // the case where we have more samples than variable.
      // In the first case we compute the (smaller) inner product matrix instead of the full covariance matrix.
      // It is known that this has the same non-zero singular values as the covariance matrix.
      // Furthermore, it is possible to compute the corresponding eigenvectors of the covariance matrix from the
      // decomposition.

      if (n < p)
      {
        // we compute the eigenvectors of the covariance matrix by computing an SVD of the
        // n x n inner product matrix 1/(n-1) X0X0^T
        MatrixType             Cov = X0 * X0.transpose() * 1.0 / (n - 1);
        SVDDoublePrecisionType SVD(Cov.cast<double>(), Eigen::ComputeThinV);
        VectorType             singularValues = SVD.singularValues().cast<ScalarType>();
        MatrixType             V = SVD.matrixV().cast<ScalarType>();

        unsigned numComponentsAboveTolerance =
          ((singularValues.array() - noiseVariance - Superclass::TOLERANCE) > 0).count();

        // there can be at most n-1 nonzero singular values in this case. Everything else must be due to numerical
        // inaccuracies
        unsigned numComponentsToKeep = std::min(numComponentsAboveTolerance, n - 1);
        // compute the pseudo inverse of the square root of the singular values
        // which is then needed to recompute the PCA basis
        VectorType singSqrt = singularValues.array().sqrt();
        VectorType singSqrtInv = VectorType::Zero(singSqrt.rows());
        for (unsigned i = 0; i < numComponentsToKeep; i++)
        {
          assert(singSqrt(i) > Superclass::TOLERANCE);
          singSqrtInv(i) = 1.0 / singSqrt(i);
        }

        if (numComponentsToKeep == 0)
        {
          throw StatisticalModelException("All the eigenvalues are below the given tolerance. Model cannot be built.");
        }

        // we recover the eigenvectors U of the full covariance matrix from the eigenvectors V of the inner product
        // matrix. We use the fact that if we decompose X as X=UDV^T, then we get X^TX = UD^2U^T and XX^T = VD^2V^T
        // (exploiting the orthogonormality of the matrix U and V from the SVD). The additional factor sqrt(n-1) is to
        // compensate for the 1/sqrt(n-1) in the formula for the covariance matrix.

        MatrixType pcaBasis = X0.transpose() * V * singSqrtInv.asDiagonal();
        pcaBasis /= sqrt(n - 1.0);
        pcaBasis.conservativeResize(Eigen::NoChange, numComponentsToKeep);


        VectorType sampleVarianceVector = singularValues.topRows(numComponentsToKeep);
        VectorType pcaVariance = (sampleVarianceVector - VectorType::Ones(numComponentsToKeep) * noiseVariance);

        auto model = StatisticalModelType::SafeCreate(representer, mu, pcaBasis, pcaVariance, noiseVariance);

        return model;
      }
      else
      {
        // we compute an SVD of the full p x p  covariance matrix 1/(n-1) X0^TX0 directly
        SVDType    SVD(X0.transpose() * X0, Eigen::ComputeThinU);
        VectorType singularValues = SVD.singularValues();
        singularValues /= (n - 1.0);
        unsigned   numComponentsToKeep = ((singularValues.array() - noiseVariance - Superclass::TOLERANCE) > 0).count();
        MatrixType pcaBasis = SVD.matrixU();

        pcaBasis.conservativeResize(Eigen::NoChange, numComponentsToKeep);

        if (numComponentsToKeep == 0)
        {
          throw StatisticalModelException("All the eigenvalues are below the given tolerance. Model cannot be built.");
        }

        VectorType sampleVarianceVector = singularValues.topRows(numComponentsToKeep);
        VectorType pcaVariance = (sampleVarianceVector - VectorType::Ones(numComponentsToKeep) * noiseVariance);
        auto       model = StatisticalModelType::SafeCreate(representer, mu, pcaBasis, pcaVariance, noiseVariance);
        return model;
      }
      break;

    case SelfAdjointEigenSolver:
    {
      // we compute the eigenvalues/eigenvectors of the full p x p  covariance matrix 1/(n-1) X0^TX0 directly

      typedef Eigen::SelfAdjointEigenSolver<MatrixType> SelfAdjointEigenSolver;
      SelfAdjointEigenSolver                            es;
      es.compute(X0.transpose() * X0);
      VectorType eigenValues =
        es.eigenvalues().reverse(); // SelfAdjointEigenSolver orders the eigenvalues in increasing order
      eigenValues /= (n - 1.0);

      unsigned   numComponentsToKeep = ((eigenValues.array() - noiseVariance - Superclass::TOLERANCE) > 0).count();
      MatrixType pcaBasis = es.eigenvectors().rowwise().reverse();
      pcaBasis.conservativeResize(Eigen::NoChange, numComponentsToKeep);

      if (numComponentsToKeep == 0)
      {
        throw StatisticalModelException("All the eigenvalues are below the given tolerance. Model cannot be built.");
      }

      VectorType sampleVarianceVector = eigenValues.topRows(numComponentsToKeep);
      VectorType pcaVariance = (sampleVarianceVector - VectorType::Ones(numComponentsToKeep) * noiseVariance);
      auto       model = StatisticalModelType::SafeCreate(representer, mu, pcaBasis, pcaVariance, noiseVariance);
      return model;
    }
    break;

    default:
      throw StatisticalModelException("Unrecognized decomposition/eigenvalue solver method.");
      return 0;
      break;
  }
}


} // namespace statismo

#endif
