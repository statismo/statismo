#ifndef __KERNELS_H
#define __KERNELS_H

#include "Representer.h"
#include "statismo/Config.h"
#include "statismo/ModelInfo.h"
#include "statismo/CommonTypes.h"
#include <vector>
#include <cmath>
#include <memory>

namespace statismo {



class ScalarValuedKernel {
public:

	virtual ~ScalarValuedKernel() {	}
	virtual double operator()(const VectorType& x,
			const VectorType& y) const = 0;

	virtual std::string GetKernelInfo() const = 0;
};


// forward declarations
class SumKernel;
class ScaledKernel;
class ProductKernel;


class MatrixValuedKernel {
public:

	MatrixValuedKernel(unsigned dim): m_dimension(dim) {}

	virtual MatrixType operator()(const VectorType& x,
			const VectorType& y) const = 0;

	virtual unsigned GetDimension() const { return m_dimension; }
	;
	virtual ~MatrixValuedKernel() {
	}

	virtual std::string GetKernelInfo() const = 0;

protected:
	unsigned m_dimension;

};






class SumKernel : public MatrixValuedKernel{
public:
	SumKernel(const MatrixValuedKernel* lhs, const MatrixValuedKernel* rhs) : MatrixValuedKernel(lhs->GetDimension()), m_lhs(lhs), m_rhs(rhs) {
		if (lhs->GetDimension() != rhs->GetDimension()) {
			throw StatisticalModelException("Kernels in SumKernel must have the same dimensionality");
		}
	}

	MatrixType operator()(const VectorType& x,const VectorType& y) const {
		return (*m_lhs)(x,y) + (*m_rhs)(x,y);
	}

	std::string GetKernelInfo() const {
		std::ostringstream os;
		os << m_lhs->GetKernelInfo() << " + " << m_rhs->GetKernelInfo();
		return os.str();
	}


private:
	const MatrixValuedKernel* m_lhs;
	const MatrixValuedKernel* m_rhs;
};


class ProductKernel : public MatrixValuedKernel{
public:
	ProductKernel(const MatrixValuedKernel* lhs, const MatrixValuedKernel* rhs) : MatrixValuedKernel(lhs->GetDimension()), m_lhs(lhs), m_rhs(rhs) {
		if (lhs->GetDimension() != rhs->GetDimension()) {
			throw StatisticalModelException("Kernels in SumKernel must have the same dimensionality");
		}

	}

	MatrixType operator()(const VectorType& x,const VectorType& y) const {
		return (*m_lhs)(x,y) * (*m_rhs)(x,y);
	}

	std::string GetKernelInfo() const {
		std::ostringstream os;
		os << m_lhs->GetKernelInfo() << " * " << m_rhs->GetKernelInfo();
		return os.str();
	}


private:
	const MatrixValuedKernel* m_lhs;
	const MatrixValuedKernel* m_rhs;
};


class ScaledKernel : public MatrixValuedKernel {
public:
	ScaledKernel(const MatrixValuedKernel* kernel, double scalingFactor) : MatrixValuedKernel(kernel->GetDimension()), m_kernel(kernel), m_scalingFactor(scalingFactor) {}

	MatrixType operator()(const VectorType& x,const VectorType& y) const {
		return (*m_kernel)(x,y) * m_scalingFactor;
	}
	std::string GetKernelInfo() const {
		std::ostringstream os;
		os << (*m_kernel).GetKernelInfo() << " * " << m_scalingFactor;
		return os.str();
	}


private:
	const MatrixValuedKernel* m_kernel;
	double m_scalingFactor;
};


class UncorrelatedMatrixValuedKernel: public MatrixValuedKernel{
public:
	UncorrelatedMatrixValuedKernel(const ScalarValuedKernel* scalarKernel, unsigned dimension):
		MatrixValuedKernel(dimension),
		m_kernel(scalarKernel),
		m_ident(MatrixType::Identity(dimension, dimension)) {}

		MatrixType operator()(const VectorType& x,
				const VectorType& y) const {

			return m_ident * (*m_kernel)(x, y);
		}

	virtual ~UncorrelatedMatrixValuedKernel() {
	}

	std::string GetKernelInfo() const {
		std::ostringstream os;
		os << "UncorrelatedMatrixValuedKernel(" << (*m_kernel).GetKernelInfo() << ", " << this->m_dimension << ")";
		return os.str();
	}

private:


	const ScalarValuedKernel* m_kernel;
	MatrixType m_ident;

};

class GaussianKernel: public ScalarValuedKernel {
public:

	 GaussianKernel(double sigma) :
		 m_sigma(sigma),
	  m_sigma2(sigma * sigma) {}


 virtual ~GaussianKernel() {}


	inline double operator()(const VectorType& x, const VectorType& y) const {
		VectorType r = x - y;
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




} // namespace statismo


#endif // __KERNELS_H
