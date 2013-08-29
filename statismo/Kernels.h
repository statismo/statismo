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
template <unsigned Dim> class SumKernel;
template <unsigned Dim> class ScaledKernel;
template <unsigned Dim> class ProductKernel;


template <unsigned Dim>
class MatrixValuedKernel {
public:

	MatrixValuedKernel() {}

	virtual MatrixType operator()(const VectorType& x,
			const VectorType& y) const = 0;

	virtual unsigned GetDimension() const { return Dim; }
	;
	virtual ~MatrixValuedKernel() {
	}

	virtual std::string GetKernelInfo() const = 0;

};






template <unsigned Dim>
class SumKernel : public MatrixValuedKernel<Dim>{
public:
	SumKernel(const MatrixValuedKernel<Dim>* lhs, const MatrixValuedKernel<Dim>* rhs) : m_lhs(lhs), m_rhs(rhs) {}

	MatrixType operator()(const VectorType& x,const VectorType& y) const {
		return (*m_lhs)(x,y) + (*m_rhs)(x,y);
	}

	std::string GetKernelInfo() const {
		std::ostringstream os;
		os << m_lhs->GetKernelInfo() << " + " << m_rhs->GetKernelInfo();
		return os.str();
	}


private:
	const MatrixValuedKernel<Dim>* m_lhs;
	const MatrixValuedKernel<Dim>* m_rhs;
};


template <unsigned Dim>
class ProductKernel : public MatrixValuedKernel<Dim>{
public:
	ProductKernel(const MatrixValuedKernel<Dim>* lhs, const MatrixValuedKernel<Dim>* rhs) : m_lhs(lhs), m_rhs(rhs) {}

	MatrixType operator()(const VectorType& x,const VectorType& y) const {
		return (*m_lhs)(x,y) * (*m_rhs)(x,y);
	}

	std::string GetKernelInfo() const {
		std::ostringstream os;
		os << m_lhs->GetKernelInfo() << " * " << m_rhs->GetKernelInfo();
		return os.str();
	}


private:
	const MatrixValuedKernel<Dim>* m_lhs;
	const MatrixValuedKernel<Dim>* m_rhs;
};


template <unsigned Dim>
class ScaledKernel : public MatrixValuedKernel<Dim> {
public:
	ScaledKernel(const MatrixValuedKernel<Dim>* kernel, double scalingFactor) : m_kernel(kernel), m_scalingFactor(scalingFactor) {}

	MatrixType operator()(const VectorType& x,const VectorType& y) const {
		return (*m_kernel)(x,y) * m_scalingFactor;
	}
	std::string GetKernelInfo() const {
		std::ostringstream os;
		os << (*m_kernel).GetKernelInfo() << " * " << m_scalingFactor;
		return os.str();
	}


private:
	const MatrixValuedKernel<Dim>* m_kernel;
	double m_scalingFactor;
};


template <unsigned Dim>
class UncorrelatedMatrixValuedKernel: public MatrixValuedKernel<Dim> {
public:
	UncorrelatedMatrixValuedKernel(const ScalarValuedKernel* scalarKernel):
		m_kernel(scalarKernel),
		m_ident(MatrixType::Identity(Dim, Dim)) {}

		MatrixType operator()(const VectorType& x,
				const VectorType& y) const {

			return m_ident * (*m_kernel)(x, y);
		}

	virtual ~UncorrelatedMatrixValuedKernel() {
	}

	std::string GetKernelInfo() const {
		std::ostringstream os;
		os << "UncorrelatedGaussianKernel(" << (*m_kernel).GetKernelInfo() << ", " << Dim << ")";
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
