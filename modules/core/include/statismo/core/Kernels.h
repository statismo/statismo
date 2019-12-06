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

#ifndef __STATIMO_CORE_KERNELS_H_
#define __STATIMO_CORE_KERNELS_H_

#include "statismo/core/CommonTypes.h"
#include "statismo/core/Config.h"
#include "statismo/core/ModelInfo.h"
#include "statismo/core/Representer.h"
#include "statismo/core/StatisticalModel.h"

#include <cmath>
#include <vector>
#include <memory>
#include <functional>

namespace statismo
{

/**
 * Base class from which all ScalarValuedKernels derive.
 */
template <class TPoint>
class ScalarValuedKernel
{
public:
  /**
   * Create a new scalar valued kernel.
   */
  ScalarValuedKernel() {}

  virtual ~ScalarValuedKernel() = default;

  /**
   * Evaluate the kernel function at the points x and y
   */
  virtual double
  operator()(const TPoint & x, const TPoint & y) const = 0;

  /**
   * Return a description of this kernel
   */
  virtual std::string
  GetKernelInfo() const = 0;
};


/**
 * Base class for all matrix valued kernels
 */
template <class TPoint>
class MatrixValuedKernel
{
public:
  /**
   * Create a new MatrixValuedKernel
   */
  MatrixValuedKernel(unsigned dim)
    : m_dimension(dim)
  {}

  /**
   * Evaluate the kernel at the points x and y
   */
  virtual MatrixType
  operator()(const TPoint & x, const TPoint & y) const = 0;

  /**
   * Return the dimensionality of the kernel (i.e. the size of the matrix)
   */
  virtual unsigned
  GetDimension() const
  {
    return m_dimension;
  };

  virtual ~MatrixValuedKernel() = default;

  /**
   * Return a description of this kernel.
   */
  virtual std::string
  GetKernelInfo() const = 0;

protected:
  unsigned m_dimension;
};

template <class T>
class StatisticalModelKernel : public MatrixValuedKernel<typename Representer<T>::PointType>
{
public:
  using RepresenterType = Representer<T>;
  using PointType = typename RepresenterType::PointType;
  using StatisticalModelType = StatisticalModel<T>;

  StatisticalModelKernel(const StatisticalModelType * model)
    : MatrixValuedKernel<PointType>(model->GetRepresenter()->GetDimensions())
    , m_statisticalModel(model)
  {}

  virtual ~StatisticalModelKernel() = default;

  inline MatrixType
  operator()(const PointType & x, const PointType & y) const override
  {
    auto m = m_statisticalModel->GetCovarianceAtPoint(x, y);
    return m;
  }

  std::string
  GetKernelInfo() const override
  {
    return "StatisticalModelKernel";
  }

private:
  const StatisticalModelType * m_statisticalModel;
};


} // namespace statismo

#endif
