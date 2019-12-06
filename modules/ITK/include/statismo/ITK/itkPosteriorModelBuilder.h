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


#ifndef __STATIMO_ITK_POSTERIOR_MODEL_BUILDER_H_
#define __STATIMO_ITK_POSTERIOR_MODEL_BUILDER_H_

#include <itkObject.h>

#include "statismo/ITK/itkDataManager.h"
#include "statismo/ITK/itkStatisticalModel.h"

#include "statismo/core/PosteriorModelBuilder.h"
#include "statismo/ITK/itkConfig.h"
#include "statismo/core/ImplWrapper.h"
#include "statismo/ITK/itkUtils.h"

#include <functional>
#include <utility>

namespace itk
{

/**
 * \brief ITK Wrapper for the statismo::PosteriorModelBuilder class.
 * \see statismo::PosteriorModelBuilder for detailed documentation.
 */
template <class T>
class PosteriorModelBuilder
  : public Object
  , public statismo::ImplWrapper<statismo::PosteriorModelBuilder<T>, statismo::SafeInitializer>
{
public:
  typedef PosteriorModelBuilder    Self;
  typedef Object                   Superclass;
  typedef SmartPointer<Self>       Pointer;
  typedef SmartPointer<const Self> ConstPointer;
  using ImplType =
    typename statismo::ImplWrapper<statismo::PosteriorModelBuilder<T>, statismo::SafeInitializer>::ImplType;


  itkNewMacro(Self);
  itkTypeMacro(PosteriorModelBuilder, Object);

  typedef statismo::DataManager<T>                   DataManagerType;
  typedef typename DataManagerType::DataItemListType DataItemListType;

  PosteriorModelBuilder() {}

  virtual ~PosteriorModelBuilder() {}


  // create statismo stuff
  typedef statismo::Representer<T>                                        RepresenterType;
  typedef typename RepresenterType::ValueType                             ValueType;
  typedef typename RepresenterType::PointType                             PointType;
  typedef typename statismo::PosteriorModelBuilder<T>::PointValueListType PointValueListType;
  typedef
    typename statismo::PosteriorModelBuilder<T>::PointValueWithCovariancePairType PointValueWithCovariancePairType;
  typedef
    typename statismo::PosteriorModelBuilder<T>::PointValueWithCovarianceListType PointValueWithCovarianceListType;
  typedef itk::StatisticalModel<T>                                                StatisticalModelType;
  typedef statismo::StatisticalModel<T>                                           StatismoStatisticalModelType;

  typename StatisticalModelType::Pointer
  BuildNewModelFromModel(const StatisticalModelType * model,
                         const PointValueListType &   pointValues,
                         double                       pointValuesNoiseVariance,
                         bool                         computeScores = true)
  {
    const auto * model_statismo = model->GetStatismoImplObj();
    using OverloadType =
      statismo::UniquePtrType<StatismoStatisticalModelType> (ImplType::*)(const StatismoStatisticalModelType * model,
                                                                          const PointValueListType & pointValues,
                                                                          double pointValuesNoiseVariance,
                                                                          bool   computeScores) const;
    auto                                   new_model_statismo = this->CallForwardImplTrans(ExceptionHandler{ *this },
                                                         static_cast<OverloadType>(&ImplType::BuildNewModelFromModel),
                                                         model_statismo,
                                                         pointValues,
                                                         pointValuesNoiseVariance,
                                                         computeScores);
    typename StatisticalModelType::Pointer model_itk = StatisticalModelType::New();
    model_itk->SetStatismoImplObj(std::move(new_model_statismo));
    return model_itk;
  }

  typename StatisticalModelType::Pointer
  BuildNewModel(DataItemListType           DataItemList,
                const PointValueListType & pointValues,
                double                     pointValuesNoiseVariance,
                double                     noiseVariance)
  {
    auto                                   model_statismo = this->CallForwardImplTrans(ExceptionHandler{ *this },
                                                     &ImplType::BuildNewModel,
                                                     model_statismo,
                                                     DataItemList,
                                                     pointValues,
                                                     pointValuesNoiseVariance,
                                                     noiseVariance);
    typename StatisticalModelType::Pointer model_itk = StatisticalModelType::New();
    model_itk->SetStatismoImplObj(std::move(model_statismo));
    return model_itk;
  }

  typename StatisticalModelType::Pointer
  BuildNewModelFromModel(const StatisticalModelType *             model,
                         const PointValueWithCovarianceListType & pointValuesWithCovariance,
                         bool                                     computeScores = true)
  {
    const StatismoStatisticalModelType * model_statismo = model->GetStatismoImplObj();

    using OverloadType = statismo::UniquePtrType<StatismoStatisticalModelType> (ImplType::*)(
      const StatismoStatisticalModelType *     model,
      const PointValueWithCovarianceListType & pointValuesWithCovariance,
      bool                                     computeScores) const;
    auto new_model_statismo = this->CallForwardImplTrans(ExceptionHandler{ *this },
                                                         static_cast<OverloadType>(&ImplType::BuildNewModelFromModel),
                                                         model_statismo,
                                                         pointValuesWithCovariance,
                                                         computeScores);

    typename StatisticalModelType::Pointer model_itk = StatisticalModelType::New();
    model_itk->SetStatismoImplObj(std::move(new_model_statismo));
    return model_itk;
  }

  typename StatisticalModelType::Pointer
  BuildNewModel(const DataItemListType &                 DataItemList,
                const PointValueWithCovarianceListType & pointValuesWithCovariance,
                double                                   noiseVariance)
  {
    auto model_statismo = this->CallForwardImplTrans(
      ExceptionHandler{ *this }, &ImplType::BuildNewModel, DataItemList, pointValuesWithCovariance, noiseVariance);
    typename StatisticalModelType::Pointer model_itk = StatisticalModelType::New();
    model_itk->SetStatismoImplObj(std::move(model_statismo));
    return model_itk;
  }
};


} // namespace itk

#endif /* ITK_POSTERIOR_MODEL_BUILDER */
