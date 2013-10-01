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
template<class TRepresenter>
class ScalarValuedKernel {
public:

	typedef typename TRepresenter::PointType PointType;

	/**
	 * Create a new scalar valued kernel.
	 */
	ScalarValuedKernel(const TRepresenter* representer) :
			m_representer(representer) {
	}

	virtual ~ScalarValuedKernel() {
	}

	/**
	 * Evaluate the kernel function at the points x and y
	 */
	virtual double operator()(const PointType& x, const PointType& y) const = 0;

	/**
	 * Return a description of this kernel
	 */
	virtual std::string GetKernelInfo() const = 0;

	/**
	 * Returns the representer.
	 */
	const TRepresenter* GetRepresenter() const { return m_representer; }

protected:
	const TRepresenter* m_representer;
};


/**
 * Base class for all matrix valued kernels
 */
template<class TRepresenter>
class MatrixValuedKernel {
public:
	typedef typename TRepresenter::PointType PointType;

	/**
	 * Create a new MatrixValuedKernel
	 */
	MatrixValuedKernel(const TRepresenter* representer, unsigned dim) :
			m_representer(representer), m_dimension(dim) {
	}

	/**
	 * Evaluate the kernel at the points x and y
	 */
	virtual MatrixType operator()(const PointType& x,
			const PointType& y) const = 0;

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

	/**
	 * Return the representer.
	 */
	const TRepresenter* GetRepresenter() const { return m_representer; }

protected:
	const TRepresenter* m_representer;
	unsigned m_dimension;

};

/**
 * A (matrix valued) kernel, which represents the sum of two matrix valued kernels.
 */
template<class TRepresenter>
class SumKernel: public MatrixValuedKernel<TRepresenter> {

public:

	typedef typename TRepresenter::PointType PointType;

	SumKernel(const MatrixValuedKernel<TRepresenter>* lhs,
			const MatrixValuedKernel<TRepresenter>* rhs) :
			MatrixValuedKernel<TRepresenter>(m_lhs->GetRepresenter(), lhs->GetDimension()),
			m_lhs(lhs),
			m_rhs(rhs) {
		if (lhs->GetDimension() != rhs->GetDimension()) {
			throw StatisticalModelException(
					"Kernels in SumKernel must have the same dimensionality");
		}
	}

	MatrixType operator()(const PointType& x, const PointType& y) const {
		return (*m_lhs)(x, y) + (*m_rhs)(x, y);
	}

	std::string GetKernelInfo() const {
		std::ostringstream os;
		os << m_lhs->GetKernelInfo() << " + " << m_rhs->GetKernelInfo();
		return os.str();
	}

private:
	const MatrixValuedKernel<TRepresenter>* m_lhs;
	const MatrixValuedKernel<TRepresenter>* m_rhs;
};



/**
 * A (matrix valued) kernel, which represents the product of two matrix valued kernels.
 */

template<class TRepresenter>
class ProductKernel: public MatrixValuedKernel<TRepresenter> {
public:

	typedef typename TRepresenter::PointType PointType;

	ProductKernel(const MatrixValuedKernel<TRepresenter>* lhs,
			const MatrixValuedKernel<TRepresenter>* rhs) :
			MatrixValuedKernel<TRepresenter>(lhs->GetDimension()), m_lhs(lhs), m_rhs(
					rhs) {
		if (lhs->GetDimension() != rhs->GetDimension()) {
			throw StatisticalModelException(
					"Kernels in SumKernel must have the same dimensionality");
		}

	}

	MatrixType operator()(const PointType& x, const PointType& y) const {
		return (*m_lhs)(x, y) * (*m_rhs)(x, y);
	}

	std::string GetKernelInfo() const {
		std::ostringstream os;
		os << m_lhs->GetKernelInfo() << " * " << m_rhs->GetKernelInfo();
		return os.str();
	}

private:
	const MatrixValuedKernel<TRepresenter>* m_lhs;
	const MatrixValuedKernel<TRepresenter>* m_rhs;
};


/**
 * A (matrix valued) kernel, which represents a scalar multiple of a matrix valued kernel.
 */

template<class TRepresenter>
class ScaledKernel: public MatrixValuedKernel<TRepresenter> {
public:

	typedef typename TRepresenter::PointType PointType;

	ScaledKernel(const MatrixValuedKernel<TRepresenter>* kernel,
			double scalingFactor) :
			MatrixValuedKernel<TRepresenter>(kernel->GetRepresenter(), kernel->GetDimension()), m_kernel(
					kernel), m_scalingFactor(scalingFactor) {
	}

	MatrixType operator()(const PointType& x, const PointType& y) const {
		return (*m_kernel)(x, y) * m_scalingFactor;
	}
	std::string GetKernelInfo() const {
		std::ostringstream os;
		os << (*m_kernel).GetKernelInfo() << " * " << m_scalingFactor;
		return os.str();
	}

private:
	const MatrixValuedKernel<TRepresenter>* m_kernel;
	double m_scalingFactor;
};


/**
 * Takes a scalar valued kernel and creates a matrix valued kernel of the given dimension.
 * The new kernel models the output components as independent, i.e. if K(x,y) is a scalar valued Kernel,
 * the matrix valued kernel becomes Id*K(x,y), where Id is an identity matrix of dimensionality d.
 */
template<class TRepresenter>
class UncorrelatedMatrixValuedKernel: public MatrixValuedKernel<TRepresenter> {
public:

	typedef typename TRepresenter::PointType PointType;

	UncorrelatedMatrixValuedKernel(
			const ScalarValuedKernel<TRepresenter>* scalarKernel,
			unsigned dimension) :
			MatrixValuedKernel<TRepresenter>(scalarKernel->GetRepresenter(),
					dimension), m_kernel(scalarKernel), m_ident(
					MatrixType::Identity(dimension, dimension)) {
	}

	MatrixType operator()(const PointType& x, const PointType& y) const {

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

	const ScalarValuedKernel<TRepresenter>* m_kernel;
	MatrixType m_ident;

};

/**
 * A scalar valued gaussian kernel.
 */
template<class TRepresenter>
class GaussianKernel: public ScalarValuedKernel<TRepresenter> {
public:

	typedef typename TRepresenter::PointType PointType;

	GaussianKernel(const TRepresenter* representer, double sigma) :
			ScalarValuedKernel<TRepresenter>(representer), m_sigma(sigma), m_sigma2(
					sigma * sigma) {
	}

	virtual ~GaussianKernel() {
	}

	inline double operator()(const PointType& x, const PointType& y) const {
		VectorType xv = this->m_representer->PointToVector(x);
		VectorType yv = this->m_representer->PointToVector(y);
		VectorType r = xv - yv;
		return exp(-r.dot(r) / m_sigma2);
	}

	std::string GetKernelInfo() const {
		std::ostringstream os;
		os << "GaussianKernel(" << m_sigma << ")";
		return os.str();
	}

private:

	double m_sigma;
	double m_sigma2;
};


/**
 * This (matrix valued) kernel represents the covariance of a given statistical model.
 */
template<class TRepresenter>
class StatisticalModelKernel: public MatrixValuedKernel<TRepresenter> {
public:

        typedef typename TRepresenter::PointType PointType;
        typedef StatisticalModel<TRepresenter> StatisticalModelType;

        StatisticalModelKernel(const StatisticalModelType* model)
        : MatrixValuedKernel<TRepresenter>(model->GetRepresenter(), model->GetRepresenter()->GetDimensions()),
          m_statisticalModel(model)
          { }


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
