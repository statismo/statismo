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
#ifndef __STATIMO_CORE_LOW_RANK_GP_MODEL_BUILDER_H_
#define __STATIMO_CORE_LOW_RANK_GP_MODEL_BUILDER_H_

#include "statismo/core/CommonTypes.h"
#include "statismo/core/Config.h"
#include "statismo/core/DataManager.h"
#include "statismo/core/Kernels.h"
#include "statismo/core/ModelInfo.h"
#include "statismo/core/ModelBuilder.h"
#include "statismo/core/Nystrom.h"
#include "statismo/core/Representer.h"
#include "statismo/core/StatisticalModel.h"
#include "statismo/core/ThreadPool.h"

#include <cmath>
#include <vector>
#include <future>
#include <thread>
#include <memory>

namespace statismo
{

/**
 * This class holds the result of the eigenfunction computation for
 * the points with index entries (lowerInd to upperInd)
 */
struct EigenfunctionComputationResult
{
  EigenfunctionComputationResult(unsigned l, unsigned u, const MatrixType & resMat)
    : lowerInd(l)
    , upperInd(u)
    , resultForPoints(resMat)
  {}

  unsigned   lowerInd;
  unsigned   upperInd;
  MatrixType resultForPoints;
};


/**
 * A model builder for building statistical models that are specified by an arbitrary Gaussian Process.
 * For details on the theoretical basis for this type of model builder, see the paper:
 *
 * "A unified approach to shape model fitting and non-rigid registration"
 * Marcel Lüthi, Christoph Jud and Thomas Vetter
 * in Proceedings of the 4th International Workshop on Machine Learning in Medical Imaging,
 * LNCS 8184, pp.66-73 Nagoya, Japan, September 2013
 *
 */
template <typename T>
class LowRankGPModelBuilder : public ModelBuilderBase<T, LowRankGPModelBuilder<T>>
{
public:
  using Superclass = ModelBuilderBase<T, LowRankGPModelBuilder<T>>;
  using ObjectFactoryType = typename Superclass::ObjectFactoryType;
  using RepresenterType = typename Superclass::RepresenterType;
  using PointType = typename RepresenterType::PointType;
  using DomainType = typename RepresenterType::DomainType;
  using DomainPointsListType = typename DomainType::DomainPointsListType;
  using MatrixValuedKernelType = MatrixValuedKernel<PointType>;
  using StatisticalModelType = typename Superclass::StatisticalModelType;

  friend ObjectFactoryType;

  /**
   * Build a new model using a zero-mean Gaussian process with given kernel.
   * \param kernel: A kernel (or covariance) function
   * \param numComponents The number of components used for the low rank approximation.
   * \param numPointsForNystrom  The number of points used for the Nystrom approximation
   *
   * \return a new statistical model representing the given Gaussian process
   */
  UniquePtrType<StatisticalModelType>
  BuildNewZeroMeanModel(const MatrixValuedKernelType & kernel,
                        unsigned                       numComponents,
                        unsigned                       numPointsForNystrom = 500) const
  {

    return BuildNewModel(m_representer->IdentitySample(), kernel, numComponents, numPointsForNystrom);
  }

  /**
   * Build a new model using a Gaussian process with given mean and kernel.
   * \param mean: A dataset that represents the mean (shape or deformation)
   * \param kernel: A kernel (or covariance) function
   * \param numComponents The number of components used for the low rank approximation.
   * \param numPointsForNystrom  The number of points used for the Nystrom approximation
   *
   * \return a new statistical model representing the given Gaussian process
   */
  UniquePtrType<StatisticalModelType>
  BuildNewModel(typename RepresenterType::DatasetConstPointerType mean,
                const MatrixValuedKernelType &                    kernel,
                unsigned                                          numComponents,
                unsigned                                          numPointsForNystrom = 500) const
  {
    auto domainPoints = m_representer->GetDomain().GetDomainPoints();
    auto numDomainPoints = m_representer->GetDomain().GetNumberOfPoints();
    auto kernelDim = kernel.GetDimension();

    auto nystrom = Nystrom<T>::SafeCreateStd(m_representer, kernel, numComponents, numPointsForNystrom);

    // We precompute the value of the eigenfunction for each domain point
    // and store it later in the pcaBasis matrix. In this way we obtain
    // a standard statismo model.
    // To save time, we parallelize over the rows
    unsigned                                                 numChunks = std::thread::hardware_concurrency() + 1;
    ThreadPool                                               pool{ numChunks - 1 };
    std::vector<std::future<EigenfunctionComputationResult>> futvec;

    for (unsigned i = 0; i <= numChunks; i++)
    {
      unsigned chunkSize =
        static_cast<unsigned>(ceil(static_cast<float>(numDomainPoints) / static_cast<float>(numChunks)));
      unsigned lowerInd = i * chunkSize;
      unsigned upperInd = std::min(static_cast<unsigned>(numDomainPoints), (i + 1) * chunkSize);

      if (lowerInd >= upperInd)
      {
        break;
      }

      futvec.emplace_back(pool.Submit([=, nys = nystrom.get(), k = &kernel]() {
        return ComputeEigenfunctionsForPoints(nys, k, numComponents, domainPoints, lowerInd, upperInd);
      }));
    }

    MatrixType pcaBasis = MatrixType::Zero(numDomainPoints * kernelDim, numComponents);

    // collect the result
    for (auto & f : futvec)
    {
      auto res = f.get();
      pcaBasis.block(res.lowerInd * kernelDim, 0, (res.upperInd - res.lowerInd) * kernelDim, pcaBasis.cols()) =
        res.resultForPoints;
    }
    futvec.clear();

    auto pcaVariance = nystrom->GetEigenvalues();
    auto mu = m_representer->SampleToSampleVector(mean);
    auto model = StatisticalModelType::SafeCreate(m_representer, mu, pcaBasis, pcaVariance, 0);

    // the model builder does not use any data. Hence the scores and the datainfo is emtpy
    MatrixType                              scores; // no scores
    typename BuilderInfo::DataInfoList      dataInfo;
    typename BuilderInfo::ParameterInfoList bi;
    bi.emplace_back(BuilderInfo::KeyValuePair("NoiseVariance", std::to_string(0)));
    bi.emplace_back(BuilderInfo::KeyValuePair("KernelInfo", kernel.GetKernelInfo()));

    // finally add meta data to the model info
    ModelInfo::BuilderInfoList biList(1, BuilderInfo{ "LowRankGPModelBuilder", dataInfo, bi });

    model->SetModelInfo(ModelInfo{ scores, biList });

    return model;
  }

private:
  /*
   * Compute the eigenfunction value at the poitns with index lowerInd - upperInd.
   * Return a result object with the given values.
   * This method is used to be able to parallelize the computations.
   */
  EigenfunctionComputationResult
  ComputeEigenfunctionsForPoints(const Nystrom<T> *             nystrom,
                                 const MatrixValuedKernelType * kernel,
                                 unsigned                       numEigenfunctions,
                                 const std::vector<PointType> & domainPts,
                                 unsigned                       lowerInd,
                                 unsigned                       upperInd) const
  {

    auto kernelDim = kernel->GetDimension();
    assert(upperInd <= domainPts.size());

    // holds the results of the computation
    MatrixType resMat = MatrixType::Zero((upperInd - lowerInd) * kernelDim, numEigenfunctions);

    // compute the nystrom extension for each point i in domainPts, for which
    // i is in the right range
    for (unsigned i = lowerInd; i < upperInd; i++)
    {
      auto pti = domainPts[i];
      resMat.block((i - lowerInd) * kernelDim, 0, kernelDim, resMat.cols()) =
        nystrom->ComputeEigenfunctionsAtPoint(pti);
    }
    return EigenfunctionComputationResult(lowerInd, upperInd, resMat);
  }

  LowRankGPModelBuilder(const RepresenterType * representer)
    : m_representer(representer)
  {}

  const RepresenterType * m_representer;
};

} // namespace statismo

#endif // __LOW_RANK_GP_MODEL_BUILDER_H_
