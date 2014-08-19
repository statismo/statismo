#ifndef __RANDSVD_H
#define __RANDSVD_H

#include <iostream>


/**
 * TODO comment and add reference to paper
 */
#include <cmath>
#include <limits>
#include <boost/random.hpp>

namespace statismo {
template <typename ScalarType>
class RandSVD {
public:

	typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> VectorType;
	typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic,
			Eigen::RowMajor> MatrixType;

	RandSVD(const MatrixType& A, unsigned k)
	{

	    unsigned n = A.rows();


		static boost::minstd_rand randgen(static_cast<unsigned>(time(0)));
		static boost::normal_distribution<> dist(0, 1);
		static boost::variate_generator<boost::minstd_rand, boost::normal_distribution<> > r(randgen, dist);

		// create gaussian random amtrix
		MatrixType Omega(n, k);
		for (unsigned i =0; i < n ; i++) {
			for (unsigned j = 0; j < k ; j++) {
				Omega(i,j) = r();
			}
		}


	    MatrixType Y = A * A.transpose() * A * Omega;
	    Eigen::FullPivHouseholderQR<MatrixType> qr(Y);
	    MatrixType Q = qr.matrixQ().leftCols(k + k);

	    MatrixType B = Q.transpose() * A;

	    typedef Eigen::JacobiSVD<MatrixType> SVDType;
	    SVDType SVD(B, Eigen::ComputeThinU);
	    MatrixType Uhat = SVD.matrixU();
	    m_D = SVD.singularValues();
	    m_U = (Q * Uhat).leftCols(k);
	}

	MatrixType matrixU() const {
		return m_U;
	}

	VectorType singularValues() const {
		return m_D;
	}


private:
	VectorType m_D;
	MatrixType m_U;
};


} // namespace statismo;
#endif // __LANCZOS_H
