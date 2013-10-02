
#ifndef __KERNELS_H
#define __KERNELS_H

#include "statismo/Config.h"
#include "statismo/ModelInfo.h"
#include "statismo/CommonTypes.h"
#include <vector>
#include <cmath>
#include <memory>

namespace statismo {

/**
 * Base class from which all ScalarValuedKernels derive.
 */
template<class TPoint>
class ScalarValuedKernel {
public:

	/**
	 * Create a new scalar valued kernel.
	 */
	ScalarValuedKernel(){	}

	virtual ~ScalarValuedKernel() {
	}

	/**
	 * Evaluate the kernel function at the points x and y
	 */
	virtual double operator()(const TPoint& x, const TPoint& y) const = 0;

	/**
	 * Return a description of this kernel
	 */
	virtual std::string GetKernelInfo() const = 0;

};


/**
 * Base class for all matrix valued kernels
 */
template<class TPoint>
class MatrixValuedKernel {
public:

	/**
	 * Create a new MatrixValuedKernel
	 */
	MatrixValuedKernel(unsigned dim) :
			m_dimension(dim) {
	}

	/**
	 * Evaluate the kernel at the points x and y
	 */
	virtual MatrixType operator()(const TPoint& x,
			const TPoint& y) const = 0;

	/**
	 * Return the dimensionality of the kernel (i.e. the size of the matrix)
	 */
	virtual unsigned GetDimension() const {
		return m_dimension;
	}
	;
	virtual ~MatrixValuedKernel() {
	}

	/**
	 * Return a description of this kernel.
	 */
	virtual std::string GetKernelInfo() const = 0;

protected:
	unsigned m_dimension;

};

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


template<class T>
class StatisticalModelKernel: public MatrixValuedKernel<typename Representer<T>::PointType > {
public:

	typedef Representer<T> RepresenterType;
	typedef typename RepresenterType::PointType PointType;
	typedef StatisticalModel<T> StatisticalModelType;

	StatisticalModelKernel(const StatisticalModelType* model) :
			MatrixValuedKernel<PointType>(model->GetRepresenter()->GetDimensions()), m_statisticalModel(model) {
	}

	virtual ~StatisticalModelKernel() {
	}

	inline MatrixType operator()(const PointType& x, const PointType& y) const {
		MatrixType m = m_statisticalModel->GetCovarianceAtPoint(x, y);
		return m;
	}

	std::string GetKernelInfo() const {
		return "StatisticalModelKernel";
	}

private:
	const StatisticalModelType* m_statisticalModel;
};

} // namespace statismo

#endif // __KERNELS_H
