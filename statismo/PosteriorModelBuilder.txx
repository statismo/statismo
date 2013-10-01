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

#include <Eigen/SVD>
#include "CommonTypes.h"
#include "PCAModelBuilder.h"

#include <iostream>

namespace statismo {

//
// PosteriorModelBuilder
//
//

template <typename T>
PosteriorModelBuilder<T>::PosteriorModelBuilder()
: Superclass()
{}

template <typename T>
typename PosteriorModelBuilder<T>::StatisticalModelType*
PosteriorModelBuilder<T>::BuildNewModel(
		const DataItemListType& sampleDataList,
		const PointValueListType& pointValues,
		double pointValuesNoiseVariance,
		double noiseVariance) const
{
	typedef PCAModelBuilder<T> PCAModelBuilderType;
	PCAModelBuilderType* modelBuilder = PCAModelBuilderType::Create();
	StatisticalModelType* model = modelBuilder->BuildNewModel(sampleDataList, noiseVariance);
	StatisticalModelType* PosteriorModel = BuildNewModelFromModel(model, pointValues, pointValuesNoiseVariance, noiseVariance);
	delete modelBuilder;
	delete model;
	return PosteriorModel;
}


template <typename T>
typename PosteriorModelBuilder<T>::StatisticalModelType*
PosteriorModelBuilder<T>::BuildNewModelFromModel(
		const StatisticalModelType* inputModel,
		const PointValueListType& pointValues,
		double sigma2,
		bool computeScores) const {

	const RepresenterType* representer = inputModel->GetRepresenter();


	// The naming of the variables correspond to those used in the paper
	// Posterior Shape Models,
	// Thomas Albrecht, Marcel Luethi, Thomas Gerig, Thomas Vetter
	//
	const MatrixType& U =  inputModel->GetOrthonormalPCABasisMatrix();
	const VectorType& mu = inputModel->GetMeanVector();

	// this method only makes sense for a proper PPCA model (e.g. the noise term is properly defined)
	// if the model has zero noise, we assume a small amount of noise
	double rho2 = std::max((double) inputModel->GetNoiseVariance(), (double) Superclass::TOLERANCE);

	unsigned dim = representer->GetDimensions();


	// build the part matrices with , considering only the points that are fixed
	//
	MatrixType U_g(pointValues.size()* dim, inputModel->GetNumberOfPrincipalComponents());
	VectorType mu_g(pointValues.size() * dim);
	VectorType s_g(pointValues.size() * dim);

	unsigned i = 0;
	for (typename PointValueListType::const_iterator it = pointValues.begin(); it != pointValues.end(); ++it) {
		VectorType val = representer->PointSampleToPointSampleVector(it->second);
		unsigned pt_id = representer->GetPointIdForPoint(it->first);
		for (unsigned d = 0; d < dim; d++) {
			U_g.row(i * dim + d) = U.row(representer->MapPointIdToInternalIdx(pt_id, d));
			mu_g[i * dim + d] = mu[representer->MapPointIdToInternalIdx(pt_id, d)];
			s_g[i * dim + d] = val[d];
		}
		i++;
	}

	VectorType D2 = inputModel->GetPCAVarianceVector().array();
	VectorType D = D2.array().sqrt();

	const MatrixType& Q_g = U_g * D.asDiagonal();
	const MatrixType& Q_gT = Q_g.transpose();

	MatrixType M = Q_gT * Q_g;
	M.diagonal() += sigma2 * VectorType::Ones(Q_g.cols());


	MatrixTypeDoublePrecision Minv = M.cast<double>().inverse();

	// the MAP solution for the latent variables (coefficients)
	VectorType coeffs = Minv.cast<ScalarType>() * Q_gT * (s_g - mu_g);

	// the MAP solution in the sample space
	VectorType mu_c = inputModel->GetRepresenter()->SampleToSampleVector(inputModel->DrawSample(coeffs));

	const VectorType& pcaVariance = inputModel->GetPCAVarianceVector();
	VectorTypeDoublePrecision pcaSdev = pcaVariance.cast<double>().array().sqrt();

	VectorType D2MinusRho = D2 - VectorType::Ones(D2.rows()) * rho2;
	// the values of D2 can be negative. We need to be careful when taking the root
	for (unsigned i = 0; i < D2MinusRho.rows(); i++) {
		D2MinusRho(i) = std::max((ScalarType) 0, D2(i));
	}
	VectorType D2MinusRhoSqrt = D2MinusRho.array().sqrt();


	typedef Eigen::JacobiSVD<MatrixTypeDoublePrecision> SVDType;
	MatrixTypeDoublePrecision innerMatrix = D2MinusRhoSqrt.cast<double>().asDiagonal() * Minv * D2MinusRhoSqrt.cast<double>().asDiagonal() * sigma2;
	SVDType svd(innerMatrix, Eigen::ComputeThinU);


	// SVD of the inner matrix 
	VectorType D_c = svd.singularValues().cast<ScalarType>();

	MatrixType U_c = U * svd.matrixU().cast<ScalarType>();

	StatisticalModelType* PosteriorModel = StatisticalModelType::Create(representer,0, mu_c, U_c, D_c, rho2);

	// Write the parameters used to build the models into the builderInfo

	typename ModelInfo::BuilderInfoList builderInfoList = inputModel->GetModelInfo().GetBuilderInfoList();

	BuilderInfo::ParameterInfoList bi;
	bi.push_back(BuilderInfo::KeyValuePair("NoiseVariance ", Utils::toString(rho2)));
	bi.push_back(BuilderInfo::KeyValuePair("FixedPointsVariance ", Utils::toString(sigma2)));
//
	BuilderInfo::DataInfoList di;

	unsigned pt_no = 0;
	for (typename PointValueListType::const_iterator it = pointValues.begin(); it != pointValues.end(); ++it) {
		VectorType val = representer->PointSampleToPointSampleVector(it->second);

		// TODO we looked up the PointId for the same point before. Having it here again is inefficient.
		unsigned pt_id = representer->GetPointIdForPoint(it->first);
		std::ostringstream keySStream;
		keySStream << "Point constraint " << pt_no;
		std::ostringstream valueSStream;
		valueSStream << "(" << pt_id << ", (";

		for (unsigned d = 0; d < dim - 1; d++) {
			valueSStream << val[d] << ",";
		}
		valueSStream << val[dim -1];
		valueSStream << "))";
		di.push_back(BuilderInfo::KeyValuePair(keySStream.str(), valueSStream.str()));
		pt_no++;
	}


	BuilderInfo builderInfo("PosteriorModelBuilder", di, bi);
	builderInfoList.push_back(builderInfo);

	MatrixType inputScores = inputModel->GetModelInfo().GetScoresMatrix();
	MatrixType scores = MatrixType::Zero(inputScores.rows(), inputScores.cols());

	if (computeScores == true) {

		// get the scores from the input model
		for (unsigned i = 0; i < inputScores.cols(); i++) {
			// reconstruct the sample from the input model and project it back into the model
			typename RepresenterType::DatasetPointerType ds = inputModel->DrawSample(inputScores.col(i));
			scores.col(i) = PosteriorModel->ComputeCoefficientsForDataset(ds);
			representer->DeleteDataset(ds);
		}
	}
	ModelInfo info(scores, builderInfoList);
	PosteriorModel->SetModelInfo(info);

	return PosteriorModel;

}

} // namespace statismo
