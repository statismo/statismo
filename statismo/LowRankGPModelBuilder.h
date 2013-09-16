#ifndef __LOW_RANK_GP_MODEL_BUILDER_H
#define __LOW_RANK_GP_MODEL_BUILDER_H

#include "Representer.h"
#include "statismo/Config.h"
#include "statismo/ModelInfo.h"
#include "statismo/ModelBuilder.h"
#include "statismo/DataManager.h"
#include "statismo/StatisticalModel.h"
#include "statismo/CommonTypes.h"
#include "statismo/Kernels.h"
#include <vector>
#include <cmath>
#include <memory>
#include <future>
#include <thread>
#include "RandSVD.h"

namespace statismo {


/*
 * This class holds the result of the eigenfunction computation for
 * the points with index entries (lowerInd to upperInd)
 */
struct ResEigenfunctionPointComputations {

	ResEigenfunctionPointComputations(unsigned _lowerInd, unsigned _upperInd,
			const MatrixType& _resMat) :
			lowerInd(_lowerInd), upperInd(_upperInd), resMatrix(_resMat) {
	}

	unsigned lowerInd;
	unsigned upperInd;
	MatrixType resMatrix;
};

/**
 * A model builder
 */

template<typename T>
class LowRankGPModelBuilder: public ModelBuilder<T> {

public:

	typedef Representer<T> RepresenterType;
	typedef ModelBuilder<T> Superclass;
	typedef typename Superclass::StatisticalModelType StatisticalModelType;

	typedef statismo::Domain<typename RepresenterType::PointType> DomainType;
	typedef typename DomainType::DomainPointsListType DomainPointsListType;

	/**
	 * Factory method to create a new PCAModelBuilder
	 */
	static LowRankGPModelBuilder* Create(const RepresenterType* representer) {
		return new LowRankGPModelBuilder(representer);
	}

	/**
	 * Destroy the object.
	 * The same effect can be achieved by deleting the object in the usual
	 * way using the c++ delete keyword.
	 */
	void Delete() {
		delete this;
	}

	/**
	 * The desctructor
	 */
	virtual ~LowRankGPModelBuilder() {
	}


	/**
	 * Computes the row of the PCABasismat for the domainPoints with index ranging from lowerInd to upperInd
	 * @param kernel A kernel function
	 * @param numEigenfunction The number of eigenfunctions
	 * @param domainPts a list of points for which the eigenfunction has to be computed
	 */
	ResEigenfunctionPointComputations computeEigenfunctionsForPoints(
			const MatrixValuedKernel* kernel, unsigned numEigenfunctions,
			unsigned numDomainPoints, const std::vector<VectorType>& xs,
			const std::vector<VectorType> & domainPts, const MatrixType& M,
			unsigned lowerInd, unsigned upperInd) const {

		unsigned m = xs.size();

		unsigned kernelDim = kernel->GetDimension();

		assert(upperInd <= domainPts.size());

		// holds the results of the computation
		MatrixType resMat = MatrixType::Zero((upperInd - lowerInd) * kernelDim,
				numEigenfunctions);

		// compute the nystrom extension for each point i in domainPts, for which
		// i is in the right range
		for (unsigned i = lowerInd; i < upperInd; i++) {

			// for every domain point x in the list, we compute the kernel vector
			// kx = (k(x, x1), ... k(x, xm))
			// since the kernel is matrix valued, kx is actually a matrix
			MatrixType kxi = MatrixType::Zero(kernelDim, m * kernelDim);

			for (unsigned j = 0; j < m; j++) {
				kxi.block(0, j * kernelDim, kernelDim, kernelDim) = (*kernel)(
						domainPts[i], xs[j]);
			}

			for (unsigned j = 0; j < numEigenfunctions; j++) {
				MatrixType x = (kxi * M.col(j));
				resMat.block((i - lowerInd) * kernelDim, j, kernelDim, 1) = x;
			}
			// add the result to the result matrix
			//resMat.block((i - lowerInd) * kernelDim, 0,  kernelDim, numEigenfunctions) =  (M  * kxi).transpose();

			// status update
			if ((i - lowerInd) % 100000 == 0 && (i - lowerInd) != 0) {
				std::cout << "processed 100'000 items out of "
						<< upperInd - lowerInd << " " << "(id = "
						<< std::this_thread::get_id() << ")" << std::endl;
			}
		}

		return ResEigenfunctionPointComputations(lowerInd, upperInd, resMat);
	}


	StatisticalModelType* BuildNewZeroMeanModel(
			const MatrixValuedKernel& kernel, unsigned numComponents,
			unsigned numPointsForNystrom = 500) const {

		VectorType zeroVec = VectorType::Zero(m_representer->GetDomain().GetNumberOfPoints() * m_representer->GetDimensions());
		typename RepresenterType::DatasetConstPointerType zeroMean = m_representer->SampleVectorToSample(zeroVec);
		return BuildNewModel(zeroMean, kernel, numComponents, numPointsForNystrom);
	}

	/**
	 * Build a new model using a Gaussian process with given mean and kernel.
	 * \param numPointsForNystrom  The number of points used in the Nystrom Approximation
	 * \param numComponents The number of principal components to be computed
	 *
	 * \return a new statistical model representing the given gaussian process
	 */
	StatisticalModelType* BuildNewModel(
			typename RepresenterType::DatasetConstPointerType mean,
			const MatrixValuedKernel& kernel, unsigned numComponents,
			unsigned numPointsForNystrom = 500) const {

		DomainType domain = m_representer->GetDomain();
		unsigned n = domain.GetNumberOfPoints();

		// convert all the points in the domain to a vector representation
		std::vector<VectorType> domainPoints;
		std::vector<VectorType> shuffledDomainPoints;
		for (typename DomainPointsListType::const_iterator it =
				domain.GetDomainPoints().begin();
				it != domain.GetDomainPoints().end(); ++it) {

			domainPoints.push_back(m_representer->PointToVector(*it));
			shuffledDomainPoints.push_back(m_representer->PointToVector(*it));
		}
		assert(domainPoints.size() == n);

		std::random_shuffle ( shuffledDomainPoints.begin(), shuffledDomainPoints.end() );

		// select a subset of the points for computing the nystrom approximation.
		std::vector<VectorType> xs;
		for (unsigned i=0; i<numPointsForNystrom; i++) {
			xs.push_back(shuffledDomainPoints[i]);
		}

		shuffledDomainPoints.clear();

		int m = xs.size();

		// compute a eigenvalue decomposition of the kernel matrix, evaluated at the points used for the
		// nystrom approximation
		unsigned kernelDim = kernel.GetDimension();
		MatrixType U; // will hold the eigenvectors (principal components)
		VectorType D; // will hold the eigenvalues (variance)
		computeKernelMatrixDecomposition(&kernel, xs, numComponents, U, D);

		// we precompute the value of the eigenfunction for each domain point
		// and store it later in the pcaBasis matrix. In this way we obtain
		// a standard statismo model.
		// To save time, we parallelize over the rows
		std::vector < std::future<ResEigenfunctionPointComputations> > resvec;
		// precompute the part of the nystrom approximation, which is independent of the domain point
		MatrixType M = sqrt(m / float(n))
				* (U.leftCols(numComponents)
						* D.topRows(numComponents).asDiagonal().inverse());

		MatrixType pcaBasis = MatrixType::Zero(n * kernelDim, numComponents);

		// we split it in 1000 chunks, which should be more than most machines have cores. so we keep all cores busy
		unsigned numChunks = 1000;
		for (unsigned i = 0; i <= numChunks; i++) {

			unsigned int chunkSize = ceil(
					domainPoints.size() / float(numChunks));
			unsigned int lowerInd = i * chunkSize;
			unsigned int upperInd = std::min(
					static_cast<unsigned>(domainPoints.size()),
					(i + 1) * chunkSize);
			if (lowerInd >= upperInd)
				break;
			resvec.push_back(
					std::async(std::launch::async,
							&LowRankGPModelBuilder<T>::computeEigenfunctionsForPoints,
							this, &kernel, numComponents, n, xs, domainPoints, M,
							lowerInd, upperInd));

		}

		// collect the result
		for (unsigned i = 0; i < resvec.size(); i++) {
			ResEigenfunctionPointComputations res = resvec[i].get();
			pcaBasis.block(res.lowerInd * kernelDim, 0,
					(res.upperInd - res.lowerInd) * kernelDim, pcaBasis.cols()) =
					res.resMatrix;
		}

		MatrixType pcaVariance = n / float(m) * D.topRows(numComponents);

		RowVectorType mu = m_representer->SampleToSampleVector(mean);

		StatisticalModelType* model = StatisticalModelType::Create(
				m_representer, 0, mu, pcaBasis, pcaVariance, 0);


		// the model builder does not use any data. Hence the scores and the datainfo is emtpy
		MatrixType scores; // no scores
		typename BuilderInfo::DataInfoList dataInfo;


		typename BuilderInfo::ParameterInfoList bi;
		bi.push_back(BuilderInfo::KeyValuePair("NoiseVariance ",		Utils::toString(0)));
		bi.push_back(BuilderInfo::KeyValuePair("KernelInfo", kernel.GetKernelInfo()));

		// finally add meta data to the model info
		BuilderInfo builderInfo("LowRankGPModelBuilder", dataInfo, bi);


		ModelInfo::BuilderInfoList biList;
		biList.push_back(builderInfo);


		ModelInfo info(scores, biList);
		model->SetModelInfo(info);

		return model;
	}


private:

	/**
	 * Compute the kernel matrix for all points given in xs and
	 * return a matrix U with the first numComponents eigenvectors and a vector D with
	 * the corresponding eigenvalues of this kernel matrix
	 */
	void computeKernelMatrixDecomposition(const MatrixValuedKernel* kernel,
			const std::vector<VectorType>& xs, unsigned numComponents,
			MatrixType& U, VectorType& D) const {
		unsigned kernelDim = kernel->GetDimension();

		double n = xs.size();
		MatrixTypeDoublePrecision K = MatrixTypeDoublePrecision::Zero(
				n * kernelDim, n * kernelDim);
		for (unsigned i = 0; i < n; ++i) {
			for (unsigned j = i; j < n; ++j) {
				MatrixType v = (*kernel)(xs[i], xs[j]);
				for (unsigned d1 = 0; d1 < kernelDim; d1++) {
					for (unsigned d2 = 0; d2 < kernelDim; d2++) {
						K(i * kernelDim + d1, j * kernelDim + d2) = v(d1, d2);
						K(j * kernelDim + d2, i * kernelDim + d1) = K(
								i * kernelDim + d1, j * kernelDim + d2);
					}
				}
			}
		}

		typedef RandSVD<double> SVDType;
		SVDType svd(K, numComponents * kernelDim);

		U = svd.matrixU().cast<ScalarType>();
		D = svd.singularValues().cast<ScalarType>();
	}

	/**
	 * constructor - only used internally
	 */
	LowRankGPModelBuilder(const RepresenterType* representer) :
			m_representer(representer) {
	}

	// purposely not implemented
	LowRankGPModelBuilder(const LowRankGPModelBuilder& orig);
	LowRankGPModelBuilder& operator=(const LowRankGPModelBuilder& rhs);

	const RepresenterType* m_representer;

};

} // namespace statismo

#endif // __LOW_RANK_GP_MODEL_BUILDER_H
