/**
 * This file is part of the statismo library.
 *
 * Author: Marcel Luethi (marcel.luethi@unibas.ch)
 *         Thomas Gerig  (thomas.gerig@unibas.ch)
 *
 * Copyright (c) 2011 University of Basel
 * All rights reserved.
 *
 * Statismo is licensed under the BSD licence (3 clause) license
 */

#ifndef KERNELCOMBINATORS_H
#define KERNELCOMBINATORS_H


#include "CommonTypes.h"
#include "Kernels.h"
#include "Representer.h"
#include "Nystrom.h"
#include <boost/unordered_map.hpp>
#include <boost/thread/mutex.hpp>


namespace statismo {



/**
 * A (matrix valued) kernel, which represents the sum of two matrix valued kernels.
 */
template<class TPoint>
class SumKernel: public MatrixValuedKernel<TPoint> {

public:

    SumKernel(const MatrixValuedKernel<TPoint>* lhs,
            const MatrixValuedKernel<TPoint>* rhs) :
            MatrixValuedKernel<TPoint>(lhs->GetDimension()),
            m_lhs(lhs),
            m_rhs(rhs) {
        if (lhs->GetDimension() != rhs->GetDimension()) {
            throw StatisticalModelException(
                    "Kernels in SumKernel must have the same dimensionality");
        }
    }

    MatrixType operator()(const TPoint& x, const TPoint& y) const {
        return (*m_lhs)(x, y) + (*m_rhs)(x, y);
    }

    std::string GetKernelInfo() const {
        std::ostringstream os;
        os << m_lhs->GetKernelInfo() << " + " << m_rhs->GetKernelInfo();
        return os.str();
    }

private:
    const MatrixValuedKernel<TPoint>* m_lhs;
    const MatrixValuedKernel<TPoint>* m_rhs;
};



/**
 * A (matrix valued) kernel, which represents the product of two matrix valued kernels.
 */

template<class TPoint>
class ProductKernel: public MatrixValuedKernel<TPoint> {
public:

    ProductKernel(const MatrixValuedKernel<TPoint>* lhs,
            const MatrixValuedKernel<TPoint>* rhs) :
            MatrixValuedKernel<TPoint>(lhs->GetDimension()), m_lhs(lhs), m_rhs(
                    rhs) {
        if (lhs->GetDimension() != rhs->GetDimension()) {
            throw StatisticalModelException(
                    "Kernels in SumKernel must have the same dimensionality");
        }

    }

    MatrixType operator()(const TPoint& x, const TPoint& y) const {
        return (*m_lhs)(x, y) * (*m_rhs)(x, y);
    }

    std::string GetKernelInfo() const {
        std::ostringstream os;
        os << m_lhs->GetKernelInfo() << " * " << m_rhs->GetKernelInfo();
        return os.str();
    }

private:
    const MatrixValuedKernel<TPoint>* m_lhs;
    const MatrixValuedKernel<TPoint>* m_rhs;
};


/**
 * A (matrix valued) kernel, which represents a scalar multiple of a matrix valued kernel.
 */

template<class TPoint>
class ScaledKernel: public MatrixValuedKernel<TPoint> {
public:

    ScaledKernel(const MatrixValuedKernel<TPoint>* kernel,
            double scalingFactor) :
            MatrixValuedKernel<TPoint>(kernel->GetDimension()), m_kernel(kernel), m_scalingFactor(scalingFactor) {
    }

    MatrixType operator()(const TPoint& x, const TPoint& y) const {
        return (*m_kernel)(x, y) * m_scalingFactor;
    }
    std::string GetKernelInfo() const {
        std::ostringstream os;
        os << (*m_kernel).GetKernelInfo() << " * " << m_scalingFactor;
        return os.str();
    }

private:
    const MatrixValuedKernel<TPoint>* m_kernel;
    double m_scalingFactor;
};


/**
 * Takes a scalar valued kernel and creates a matrix valued kernel of the given dimension.
 * The new kernel models the output components as independent, i.e. if K(x,y) is a scalar valued Kernel,
 * the matrix valued kernel becomes Id*K(x,y), where Id is an identity matrix of dimensionality d.
 */
template<class TPoint>
class UncorrelatedMatrixValuedKernel: public MatrixValuedKernel<TPoint> {
public:

    UncorrelatedMatrixValuedKernel(
            const ScalarValuedKernel<TPoint>* scalarKernel,
            unsigned dimension) :
            MatrixValuedKernel<TPoint>(	dimension), m_kernel(scalarKernel),
            m_ident(MatrixType::Identity(dimension, dimension)) {
    }

    MatrixType operator()(const TPoint& x, const TPoint& y) const {

        return m_ident * (*m_kernel)(x, y);
    }

    virtual ~UncorrelatedMatrixValuedKernel() {
    }

    std::string GetKernelInfo() const {
        std::ostringstream os;
        os << "UncorrelatedMatrixValuedKernel(" << (*m_kernel).GetKernelInfo()
                << ", " << this->m_dimension << ")";
        return os.str();
    }

private:

    const ScalarValuedKernel<TPoint>* m_kernel;
    MatrixType m_ident;

};


/**
 * Base class for defining a tempering function for the SpatiallyVaryingKernel
 */
template <class TPoint>
class TemperingFunction : public std::unary_function<TPoint, double> {
public:
    virtual double operator()(const TPoint& pt) const = 0;
    virtual ~TemperingFunction() {}
};



}

#endif // KERNELCOMBINATORS_H
