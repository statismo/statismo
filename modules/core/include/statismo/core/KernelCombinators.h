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

#ifndef __STATIMO_CORE_KERNEL_COMBINATORS_H_
#define __STATIMO_CORE_KERNEL_COMBINATORS_H_

#include "statismo/core/CommonTypes.h"
#include "statismo/core/Kernels.h"
#include "statismo/core/Nystrom.h"
#include "statismo/core/Representer.h"
#include "statismo/core/Hash.h"
#include "statismo/core/SafeContainer.h"

#include <memory>
#include <sstream>

namespace statismo
{

/**
 * A (matrix valued) kernel, which represents the sum of two matrix valued kernels.
 */
template <class TPoint>
class SumKernel : public MatrixValuedKernel<TPoint>
{
public:
  using MatrixValuedKernelType = MatrixValuedKernel<TPoint>;

  SumKernel(const MatrixValuedKernelType * lhs, const MatrixValuedKernelType * rhs)
    : MatrixValuedKernelType(lhs->GetDimension())
    , m_lhs(lhs)
    , m_rhs(rhs)
  {
    if (lhs->GetDimension() != rhs->GetDimension())
    {
      throw StatisticalModelException("Kernels in SumKernel must have the same dimensionality",
                                      Status::BAD_INPUT_ERROR);
    }
  }

  MatrixType
  operator()(const TPoint & x, const TPoint & y) const
  {
    return (*m_lhs)(x, y) + (*m_rhs)(x, y);
  }

  std::string
  GetKernelInfo() const
  {
    return m_lhs->GetKernelInfo() + " + " + m_rhs->GetKernelInfo();
  }

private:
  const MatrixValuedKernelType * m_lhs;
  const MatrixValuedKernelType * m_rhs;
};


/**
 * A (matrix valued) kernel, which represents the product of two matrix valued kernels.
 */
template <class TPoint>
class ProductKernel : public MatrixValuedKernel<TPoint>
{

public:
  using MatrixValuedKernelType = MatrixValuedKernel<TPoint>;

  ProductKernel(const MatrixValuedKernelType * lhs, const MatrixValuedKernelType * rhs)
    : MatrixValuedKernelType(lhs->GetDimension())
    , m_lhs(lhs)
    , m_rhs(rhs)
  {
    if (lhs->GetDimension() != rhs->GetDimension())
    {
      throw StatisticalModelException("Kernels in SumKernel must have the same dimensionality");
    }
  }

  MatrixType
  operator()(const TPoint & x, const TPoint & y) const
  {
    return (*m_lhs)(x, y) * (*m_rhs)(x, y);
  }

  std::string
  GetKernelInfo() const
  {
    return m_lhs->GetKernelInfo() + " * " + m_rhs->GetKernelInfo();
  }

private:
  const MatrixValuedKernelType * m_lhs;
  const MatrixValuedKernelType * m_rhs;
};


/**
 * A (matrix valued) kernel, which represents a scalar multiple of a matrix valued kernel.
 */
template <class TPoint>
class ScaledKernel : public MatrixValuedKernel<TPoint>
{
public:
  using MatrixValuedKernelType = MatrixValuedKernel<TPoint>;

  ScaledKernel(const MatrixValuedKernelType * kernel, double scalingFactor)
    : MatrixValuedKernelType(kernel->GetDimension())
    , m_kernel(kernel)
    , m_scalingFactor(scalingFactor)
  {}

  MatrixType
  operator()(const TPoint & x, const TPoint & y) const
  {
    return (*m_kernel)(x, y) * m_scalingFactor;
  }

  std::string
  GetKernelInfo() const
  {
    return m_kernel->GetKernelInfo() + " * " + std::to_string(m_scalingFactor);
  }

private:
  const MatrixValuedKernelType * m_kernel;
  double                         m_scalingFactor;
};


/**
 * Takes a scalar valued kernel and creates a matrix valued kernel of the given dimension.
 * The new kernel models the output components as independent, i.e. if K(x,y) is a scalar valued Kernel,
 * the matrix valued kernel becomes Id*K(x,y), where Id is an identity matrix of dimensionality d.
 */
template <class TPoint>
class UncorrelatedMatrixValuedKernel : public MatrixValuedKernel<TPoint>
{
public:
  using MatrixValuedKernelType = MatrixValuedKernel<TPoint>;

  UncorrelatedMatrixValuedKernel(const ScalarValuedKernel<TPoint> * scalarKernel, unsigned dimension)
    : MatrixValuedKernelType(dimension)
    , m_kernel(scalarKernel)
    , m_ident(MatrixType::Identity(dimension, dimension))
  {}

  MatrixType
  operator()(const TPoint & x, const TPoint & y) const
  {

    return m_ident * (*m_kernel)(x, y);
  }

  std::string
  GetKernelInfo() const
  {
    std::ostringstream os;
    os << "UncorrelatedMatrixValuedKernel(" << (*m_kernel).GetKernelInfo() << ", " << this->m_dimension << ")";
    return os.str();
  }

private:
  const ScalarValuedKernel<TPoint> * m_kernel;
  MatrixType                         m_ident;
};


/**
 * Base class for defining a tempering function for the SpatiallyVaryingKernel
 */
template <class TPoint>
class TemperingFunction
{
public:
  virtual double
  operator()(const TPoint & pt) const = 0;
  virtual ~TemperingFunction() {}
};

/**
 * Spatially-varing kernel, as described in the paper:
 *
 * T. Gerig, K. Shahim, M. Reyes, T. Vetter, M. Luethi
 * Spatially varying registration using gaussian processes
 * Miccai 2014
 */
template <class T>
class SpatiallyVaryingKernel : public MatrixValuedKernel<typename Representer<T>::PointType>
{

public:
  using RepresenterType = Representer<T>;
  using PointType = typename RepresenterType::PointType;

  /**
   * @brief Make a given kernel spatially varying according to the given tempering function
   * @param representer, A representer which defines the domain over which the approximation is done
   * @param kernel The kernel that is made spatially adaptive
   * @param eta The tempering function that defines the amount of tempering for each point in the domain
   * @param numEigenfunctions The number of eigenfunctions to be used for the approximation
   * @param numberOfPointsForApproximation The number of points used for the nystrom approximation
   * @param cacheValues Cache result of eigenfunction computations. Greatly speeds up the computation.
   */
  SpatiallyVaryingKernel(const RepresenterType *               representer,
                         const MatrixValuedKernel<PointType> & kernel,
                         const TemperingFunction<PointType> &  eta,
                         unsigned                              numEigenfunctions,
                         unsigned                              numberOfPointsForApproximation = 0,
                         bool                                  cacheValues = true)
    : m_representer(representer)
    , m_eta(eta)
    , m_nystrom(Nystrom<T>::SafeCreateStd(representer,
                                          kernel,
                                          numEigenfunctions,
                                          numberOfPointsForApproximation == 0 ? numEigenfunctions * 2
                                                                              : numberOfPointsForApproximation))
    , m_eigenvalues(m_nystrom->GetEigenvalues())
    , m_doCacheValues(cacheValues)
    , MatrixValuedKernel<PointType>(kernel.GetDimension())
  {}

  inline MatrixType
  operator()(const PointType & x, const PointType & y) const
  {

    MatrixType sum = MatrixType::Zero(this->m_dimension, this->m_dimension);

    auto etaX = m_eta(x);
    auto etaY = m_eta(y);

    auto phisAtX = PhiAtPoint(x);
    auto phisAtY = PhiAtPoint(y);

    double largestTemperedEigenvalue = std::pow(m_eigenvalues(0), (etaX + etaY) / 2);

    for (unsigned i = 0; i < m_eigenvalues.size(); ++i)
    {
      double temperedEigenvalue = std::pow(m_eigenvalues(i), (etaX + etaY) / 2);

      // Ignore too small eigenvalues, as they don't contribute much.
      // (the eigenvalues are ordered, all the following are smaller and can also be ignored)
      if (temperedEigenvalue / largestTemperedEigenvalue < 1e-6)
      {
        break;
      }
      else
      {
        sum += phisAtX.col(i) * phisAtY.col(i).transpose() * temperedEigenvalue;
      }
    }
    // normalize such that the largest eigenvalue is unaffected by the tempering
    double normalizationFactor = largestTemperedEigenvalue / m_eigenvalues(0);
    sum *= 1.0 / normalizationFactor;
    return sum;
  }

  virtual ~SpatiallyVaryingKernel() = default;

  std::string
  GetKernelInfo() const
  {
    return "SpatiallyVaryingKernel";
  }

private:
  using CacheType = SafeUnorderedMap<statismo::VectorType, statismo::MatrixType, Hash<statismo::VectorType>>;

  // returns a d x n matrix holding the value of all n eigenfunctions evaluated at the given point.
  statismo::MatrixType
  PhiAtPoint(const PointType & pt) const
  {
    statismo::MatrixType v;
    if (m_doCacheValues)
    {
      // we need to convert the point to a vector, as the function hash_value (required by boost)
      // is not defined for an arbitrary point.
      auto ptAsVec = this->m_representer->PointToVector(pt);
      // TODO: Create thread-safe data structure in statismo instead
      //       of putting the burden of thread-safety on science related
      //       classes
      if (!m_phiCache.Find(ptAsVec, v))
      {
        v = m_nystrom->ComputeEigenfunctionsAtPoint(pt);
        m_phiCache.Insert(std::make_pair(ptAsVec, v));
      }
    }
    else
    {
      v = m_nystrom->ComputeEigenfunctionsAtPoint(pt);
    }
    return v;
  }

  //
  // members

  const RepresenterType *              m_representer;
  std::unique_ptr<Nystrom<T>>          m_nystrom;
  statismo::VectorType                 m_eigenvalues;
  const TemperingFunction<PointType> & m_eta;
  bool                                 m_doCacheValues;
  mutable CacheType                    m_phiCache;
};


} // namespace statismo

#endif // KERNELCOMBINATORS_H
