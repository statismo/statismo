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

#ifndef __STATIMO_ITK_STATISTICAL_MODEL_H_
#define __STATIMO_ITK_STATISTICAL_MODEL_H_

#include <itkObject.h>
#include <itkObjectFactory.h>

#include <vnl/vnl_matrix.h>
#include <vnl/vnl_vector.h>

#include "statismo/core/ModelInfo.h"
#include "statismo/core/Representer.h"
#include "statismo/core/StatisticalModel.h"
#include "statismo/ITK/itkConfig.h"
#include "statismo/core/ImplWrapper.h"
#include "statismo/ITK/itkUtils.h"

#include <utility>
#include <functional>

namespace itk
{

/**
 * \brief ITK Wrapper for the statismo::StatisticalModel class.
 * \see statismo::StatisticalModel for detailed documentation.
 */
template <class T>
class StatisticalModel
  : public Object
  , public statismo::ImplWrapper<statismo::StatisticalModel<T>>
{
public:
  typedef StatisticalModel         Self;
  typedef Object                   Superclass;
  typedef SmartPointer<Self>       Pointer;
  typedef SmartPointer<const Self> ConstPointer;

  using ImplType = typename statismo::ImplWrapper<statismo::StatisticalModel<T>>::ImplType;


  itkNewMacro(Self);
  itkTypeMacro(StatisticalModel, Object);

  typedef statismo::Representer<T> RepresenterType;


  typedef typename statismo::DataManager<T>::DataItemType DataItemType;

  typedef vnl_matrix<statismo::ScalarType> MatrixType;
  typedef vnl_vector<statismo::ScalarType> VectorType;


  StatisticalModel() {}

  virtual ~StatisticalModel()
  {
    /*if (m_impl)
    {
      delete m_impl;
    }*/
  }


  typedef typename RepresenterType::DatasetPointerType      DatasetPointerType;
  typedef typename RepresenterType::DatasetConstPointerType DatasetConstPointerType;

  typedef typename RepresenterType::ValueType ValueType;
  typedef typename RepresenterType::PointType PointType;

  typedef typename statismo::StatisticalModel<T>::PointValuePairType PointValuePairType;
  typedef typename statismo::StatisticalModel<T>::PointValueListType PointValueListType;

  typedef typename statismo::StatisticalModel<T>::PointCovarianceMatrixType        PointCovarianceMatrixType;
  typedef typename statismo::StatisticalModel<T>::PointValueWithCovariancePairType PointValueWithCovariancePairType;
  typedef typename statismo::StatisticalModel<T>::PointValueWithCovarianceListType PointValueWithCovarianceListType;

  typedef typename statismo::StatisticalModel<T>::DomainType DomainType;


  // TODO: wrap StatisticalModel* BuildReducedVarianceModel( double pcvar );

  const RepresenterType *
  GetRepresenter() const
  {
    return this->CallForwardImplTrans(ExceptionHandler{ *this }, &ImplType::GetRepresenter);
  }

  const DomainType &
  GetDomain() const
  {
    return this->CallForwardImplTrans(ExceptionHandler{ *this }, &ImplType::GetDomain);
  }

  DatasetPointerType
  DrawMean() const
  {
    return this->CallForwardImplTrans(ExceptionHandler{ *this }, &ImplType::DrawMean);
  }

  ValueType
  DrawMeanAtPoint(const PointType & pt) const
  {
    using OverloadType = ValueType (ImplType::*)(const PointType &) const;
    return this->CallForwardImplTrans(
      ExceptionHandler{ *this }, static_cast<OverloadType>(&ImplType::DrawMeanAtPoint), pt);
  }

  ValueType
  DrawMeanAtPoint(unsigned ptid) const
  {
    using OverloadType = ValueType (ImplType::*)(unsigned) const;
    return this->CallForwardImplTrans(
      ExceptionHandler{ *this }, static_cast<OverloadType>(&ImplType::DrawMeanAtPoint), ptid);
    // return callstatismoImpl(std::bind(static_cast<functype>(&ImplType::DrawMeanAtPoint), this->m_impl, ptid));
  }

  DatasetPointerType
  DrawSample(const VectorType & coeffs, bool addNoise = false) const
  {
    using OverloadType = DatasetPointerType (ImplType::*)(const statismo::VectorType &, bool) const;
    /*return callstatismoImpl(
      std::bind(static_cast<functype>(&ImplType::DrawSample), this->m_impl, fromVnlVector(coeffs), addNoise));*/
    return this->CallForwardImplTrans(
      ExceptionHandler{ *this }, static_cast<OverloadType>(&ImplType::DrawSample), fromVnlVector(coeffs), addNoise);
  }

  DatasetPointerType
  DrawSample(bool addNoise = false) const
  {
    using OverloadType = DatasetPointerType (ImplType::*)(bool) const;
    return this->CallForwardImplTrans(
      ExceptionHandler{ *this }, static_cast<OverloadType>(&ImplType::DrawSample), addNoise);
    // return callstatismoImpl(std::bind(static_cast<functype>(&ImplType::DrawSample), this->m_impl, addNoise));
  }

  DatasetPointerType
  DrawPCABasisSample(unsigned componentNumber) const
  {
    using OverloadType = DatasetPointerType (ImplType::*)(unsigned) const;
    return this->CallForwardImplTrans(
      ExceptionHandler{ *this }, static_cast<OverloadType>(&ImplType::DrawPCABasisSample), componentNumber);

    // return callstatismoImpl(
    // std::bind(static_cast<functype>(&ImplType::DrawPCABasisSample), this->m_impl, componentNumber));
  }

  ValueType
  DrawSampleAtPoint(const VectorType & coeffs, const PointType & pt, bool addNoise = false) const
  {
    using OverloadType = ValueType (ImplType::*)(const statismo::VectorType &, const PointType &, bool) const;
    return this->CallForwardImplTrans(ExceptionHandler{ *this },
                                      static_cast<OverloadType>(&ImplType::DrawSampleAtPoint),
                                      fromVnlVector(coeffs),
                                      pt,
                                      addNoise);
    // return callstatismoImpl(std::bind(
    // static_cast<functype>(&ImplType::DrawSampleAtPoint), this->m_impl, fromVnlVector(coeffs), pt, addNoise));
  }

  ValueType
  DrawSampleAtPoint(const VectorType & coeffs, unsigned ptid, bool addNoise = false) const
  {
    using OverloadType = ValueType (ImplType::*)(const statismo::VectorType &, unsigned, bool) const;
    return this->CallForwardImplTrans(ExceptionHandler{ *this },
                                      static_cast<OverloadType>(&ImplType::DrawSampleAtPoint),
                                      fromVnlVector(coeffs),
                                      ptid,
                                      addNoise);

    //  return callstatismoImpl(std::bind(
    // static_cast<functype>(&ImplType::DrawSampleAtPoint), this->m_impl, fromVnlVector(coeffs), ptid, addNoise));
  }


  VectorType
  ComputeCoefficients(DatasetConstPointerType ds) const
  {
    return toVnlVector(this->CallForwardImplTrans(ExceptionHandler{ *this }, &ImplType::ComputeCoefficients, ds));

    // return toVnlVector(callstatismoImpl(std::bind(&ImplType::ComputeCoefficients, this->m_impl, ds)));
  }

  double
  ComputeLogProbability(DatasetConstPointerType ds) const
  {
    return this->CallForwardImplTrans(ExceptionHandler{ *this }, &ImplType::ComputeLogProbability, ds);
    // return callstatismoImpl(std::bind(&ImplType::ComputeLogProbability, this->m_impl, ds));
  }

  double
  ComputeProbability(DatasetConstPointerType ds) const
  {
    return this->CallForwardImplTrans(ExceptionHandler{ *this }, &ImplType::ComputeProbability, ds);
    // return callstatismoImpl(std::bind(&ImplType::ComputeProbability, this->m_impl, ds));
  }

  double
  ComputeLogProbabilityOfCoefficients(const VectorType & coeffs) const
  {
    return this->CallForwardImplTrans(
      ExceptionHandler{ *this }, &ImplType::ComputeLogProbabilityOfCoefficients, fromVnlVector(coeffs));
    // return callstatismoImpl(
    // std::bind(&ImplType::ComputeLogProbabilityOfCoefficients, this->m_impl, fromVnlVector(coeffs)));
  }

  double
  ComputeProbabilityOfCoefficients(const VectorType & coeffs) const
  {
    return this->CallForwardImplTrans(
      ExceptionHandler{ *this }, &ImplType::ComputeProbabilityOfCoefficients, fromVnlVector(coeffs));
  }

  double
  ComputeMahalanobisDistance(DatasetConstPointerType ds) const
  {
    return this->CallForwardImplTrans(ExceptionHandler{ *this }, &ImplType::ComputeMahalanobisDistance, ds);
    // return callstatismoImpl(std::bind(&ImplType::ComputeMahalanobisDistance, this->m_impl, ds));
  }

  VectorType
  ComputeCoefficientsForPointValues(const PointValueListType & pvlist, double variance) const
  {
    using OverloadType = statismo::VectorType (ImplType::*)(const PointValueListType &, double) const;
    return toVnlVector(
      this->CallForwardImplTrans(ExceptionHandler{ *this },
                                 static_cast<OverloadType>(&ImplType::ComputeCoefficientsForPointValues),
                                 pvlist,
                                 variance));
    // return toVnlVector(callstatismoImpl(
    // std::bind(static_cast<functype>(&ImplType::ComputeCoefficientsForPointValues), this->m_impl, pvlist, variance)));
  }

  VectorType
  ComputeCoefficientsForPointValuesWithCovariance(const PointValueWithCovarianceListType & pvclist) const
  {
    using OverloadType = statismo::VectorType (ImplType::*)(const PointValueWithCovarianceListType &) const;
    return toVnlVector(
      this->CallForwardImplTrans(ExceptionHandler{ *this },
                                 static_cast<OverloadType>(&ImplType::ComputeCoefficientsForPointValuesWithCovariance),
                                 pvclist));
    // return toVnlVector(callstatismoImpl(std::bind(
    // static_cast<functype>(&ImplType::ComputeCoefficientsForPointValuesWithCovariance), this->m_impl, pvclist)));
  }

  unsigned
  GetNumberOfPrincipalComponents() const
  {
    return this->CallForwardImplTrans(ExceptionHandler{ *this }, &ImplType::GetNumberOfPrincipalComponents);
    // return callstatismoImpl(std::bind(&ImplType::GetNumberOfPrincipalComponents, this->m_impl));
  }

  float
  GetNoiseVariance() const
  {
    return this->CallForwardImplTrans(ExceptionHandler{ *this }, &ImplType::GetNoiseVariance);
    // return callstatismoImpl(std::bind(&ImplType::GetNoiseVariance, this->m_impl));
  }

  MatrixType
  GetCovarianceAtPoint(const PointType & pt1, const PointType & pt2) const
  {
    using OverloadType = statismo::MatrixType (ImplType::*)(const PointType &, const PointType &) const;
    return toVnlMatrix(this->CallForwardImplTrans(
      ExceptionHandler{ *this }, static_cast<OverloadType>(&ImplType::GetCovarianceAtPoint), pt1, pt2));
    // return toVnlMatrix(
    // callstatismoImpl(std::bind(static_cast<functype>(&ImplType::GetCovarianceAtPoint), this->m_impl, pt1, pt2)));
  }

  MatrixType
  GetCovarianceAtPoint(unsigned ptid1, unsigned ptid2) const
  {
    using OverloadType = statismo::MatrixType (ImplType::*)(unsigned, unsigned) const;
    return toVnlMatrix(this->CallForwardImplTrans(
      ExceptionHandler{ *this }, static_cast<OverloadType>(&ImplType::GetCovarianceAtPoint), ptid1, ptid2));
    // return toVnlMatrix(
    // callstatismoImpl(std::bind(static_cast<functype>(&ImplType::GetCovarianceAtPoint), this->m_impl, ptid1, ptid2)));
  }

  MatrixType
  GetJacobian(const PointType & pt) const
  {
    using OverloadType = statismo::MatrixType (ImplType::*)(const PointType &) const;
    return toVnlMatrix(
      this->CallForwardImplTrans(ExceptionHandler{ *this }, static_cast<OverloadType>(&ImplType::GetJacobian), pt));
    // return toVnlMatrix(callstatismoImpl(std::bind(static_cast<functype>(&ImplType::GetJacobian), this->m_impl, pt)));
  }

  MatrixType
  GetJacobian(unsigned ptId) const
  {
    using OverloadType = statismo::MatrixType (ImplType::*)(unsigned) const;
    return toVnlMatrix(
      this->CallForwardImplTrans(ExceptionHandler{ *this }, static_cast<OverloadType>(&ImplType::GetJacobian), ptId));
    // return toVnlMatrix(callstatismoImpl(std::bind(static_cast<functype>(&ImplType::GetJacobian), this->m_impl,
    // ptId)));
  }

  MatrixType
  GetPCABasisMatrix() const
  {
    return toVnlMatrix(this->CallForwardImplTrans(ExceptionHandler{ *this }, &ImplType::GetPCABasisMatrix));
    // return toVnlMatrix(callstatismoImpl(std::bind(&ImplType::GetPCABasisMatrix, this->m_impl)));
  }

  MatrixType
  GetOrthonormalPCABasisMatrix() const
  {
    return toVnlMatrix(this->CallForwardImplTrans(ExceptionHandler{ *this }, &ImplType::GetOrthonormalPCABasisMatrix));
    // return toVnlMatrix(callstatismoImpl(std::bind(&ImplType::GetOrthonormalPCABasisMatrix, this->m_impl)));
  }

  VectorType
  GetPCAVarianceVector() const
  {
    return toVnlVector(this->CallForwardImplTrans(ExceptionHandler{ *this }, &ImplType::GetPCAVarianceVector));
    // return toVnlVector(callstatismoImpl(std::bind(&ImplType::GetPCAVarianceVector, this->m_impl)));
  }

  VectorType
  GetMeanVector() const
  {
    return toVnlVector(this->CallForwardImplTrans(ExceptionHandler{ *this }, &ImplType::GetMeanVector));
    // return toVnlVector(callstatismoImpl(std::bind(&ImplType::GetMeanVector, this->m_impl)));
  }

  const statismo::ModelInfo &
  GetModelInfo() const
  {
    return this->CallForwardImplTrans(ExceptionHandler{ *this }, &ImplType::GetModelInfo);
  }

private:
  static MatrixType
  toVnlMatrix(const statismo::MatrixType & M)
  {
    return MatrixType(M.data(), M.rows(), M.cols());
  }

  static VectorType
  toVnlVector(const statismo::VectorType & v)
  {
    return VectorType(v.data(), v.rows());
  }

  static statismo::VectorType
  fromVnlVector(const VectorType & v)
  {
    return Eigen::Map<const statismo::VectorType>(v.data_block(), v.size());
  }
};


} // namespace itk

#endif /* ITKSTATISTICALMODEL_H_ */
