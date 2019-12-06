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

#ifndef __STATIMO_CORE_REDUCED_VARIANCE_MODEL_BUILDER_H_
#define __STATIMO_CORE_REDUCED_VARIANCE_MODEL_BUILDER_H_

#include "statismo/core/Config.h"
#include "statismo/core/CommonTypes.h"
#include "statismo/core/DataManager.h"
#include "statismo/core/ModelBuilder.h"
#include "statismo/core/ModelInfo.h"
#include "statismo/core/Utils.h"
#include "statismo/core/StatisticalModel.h"

#include <vector>
#include <memory>

namespace statismo
{

/**
 * \brief Builds a new model which retains only the specified total variance
 *
 */
template <typename Representer>
class ReducedVarianceModelBuilder : public ModelBuilderBase<Representer, ReducedVarianceModelBuilder<Representer>>
{

public:
  using Superclass = ModelBuilderBase<Representer, ReducedVarianceModelBuilder<Representer>>;
  using StatisticalModelType = typename Superclass::StatisticalModelType;
  friend typename Superclass::ObjectFactoryType;

  /**
   * Build a new model from the given model, which retains only the leading principal components
   *
   * \param model A statistical model.
   * \param numberOfPrincipalComponents,
   * \return a new statistical model
   *
   * \warning The returned model needs to be explicitly deleted by the user of this method.
   */
  UniquePtrType<StatisticalModelType>
  BuildNewModelWithLeadingComponents(const StatisticalModelType * model, unsigned numberOfPrincipalComponents) const;


  /**
   * Build a new model from the given model, which retains only the specified variance
   *
   * \param model A statistical model.
   * \param totalVariance, The fraction of the variance to be retained
   * \return a new statistical model
   *
   * \warning The returned model needs to be explicitly deleted by the user of this method.
   */
  UniquePtrType<StatisticalModelType>
  BuildNewModelWithVariance(const StatisticalModelType * model, double totalVariance) const;


  [[deprecated]] UniquePtrType<StatisticalModelType>
  BuildNewModelFromModel(const StatisticalModelType * model, double totalVariance) const;


private:
  ReducedVarianceModelBuilder() = default;
};


} // namespace statismo

#include "ReducedVarianceModelBuilder.hxx"

#endif
