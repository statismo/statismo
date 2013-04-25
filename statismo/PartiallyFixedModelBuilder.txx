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
// PartiallyFixedModelBuilder
//
//

template <typename Representer>
PartiallyFixedModelBuilder<Representer>::PartiallyFixedModelBuilder()
: Superclass()
{}

template <typename Representer>
typename PartiallyFixedModelBuilder<Representer>::StatisticalModelType*
PartiallyFixedModelBuilder<Representer>::BuildNewModel(
		const SampleDataStructureListType& sampleDataList,
		const PointValueListType& pointValues,
		double pointValuesNoiseVariance,
		double noiseVariance) const
{
	typedef PCAModelBuilder<Representer> PCAModelBuilderType;
	PCAModelBuilderType* modelBuilder = PCAModelBuilderType::Create();
	StatisticalModelType* model = modelBuilder->BuildNewModel(sampleDataList, noiseVariance);
	StatisticalModelType* partiallyFixedModel = BuildNewModelFromModel(model, pointValues, pointValuesNoiseVariance, noiseVariance);
	delete modelBuilder;
	delete model;
	return partiallyFixedModel;
}


template <typename Representer>
typename PartiallyFixedModelBuilder<Representer>::StatisticalModelType*
PartiallyFixedModelBuilder<Representer>::BuildNewModelFromModel(
		const StatisticalModelType* inputModel,
		const PointValueListType& pointValues,
		double pointValuesNoiseVariance,
		bool computeScores) const {

	const Representer* representer = inputModel->GetRepresenter();

	const MatrixType& pcaBasisMatrix =  inputModel->GetOrthonormalPCABasisMatrix();
	const VectorType& meanVector = inputModel->GetMeanVector();

	// this method only makes sense for a proper PPCA model (e.g. the noise term is properly defined)
	// if the model has zero noise, we assume a small amount of noise
	double noiseVariance = std::max((double) inputModel->GetNoiseVariance(), (double) Superclass::TOLERANCE);

	unsigned dim = Representer::GetDimensions();


	// build the part matrices with , considering only the points that are fixed
	//
	MatrixType PCABasisPart(pointValues.size()* dim, inputModel->GetNumberOfPrincipalComponents());
	VectorType muPart(pointValues.size() * dim);
	VectorType samplePart(pointValues.size() * dim);

	unsigned i = 0;
	for (typename PointValueListType::const_iterator it = pointValues.begin(); it != pointValues.end(); ++it) {
		VectorType val = representer->PointSampleToPointSampleVector(it->second);
		unsigned pt_id = representer->GetPointIdForPoint(it->first);
		for (unsigned d = 0; d < dim; d++) {
			PCABasisPart.row(i * dim + d) = pcaBasisMatrix.row(Representer::MapPointIdToInternalIdx(pt_id, d));
			muPart[i * dim + d] = meanVector[Representer::MapPointIdToInternalIdx(pt_id, d)];
			samplePart[i * dim + d] = val[d];
		}
		i++;
	}

	VectorType D2 = inputModel->GetPCAVarianceVector().array() - pointValuesNoiseVariance;
	// the values of D2 can be negative. We need to be careful when taking the root
	for (unsigned i = 0; i < D2.rows(); i++) {
		D2(i) = std::max((ScalarType) 0, D2(i));
	}
	VectorType D = D2.array().sqrt();

	// the names of the matrices are those used in Bishop, Pattern recognition and Machine learning (PRML),
	// chapter 12, on which this implementation is based.
	const MatrixType& W = PCABasisPart * D.asDiagonal();
	const MatrixType& WT = W.transpose();

	MatrixType M = WT * W;
	M.diagonal() += pointValuesNoiseVariance * VectorType::Ones(W.cols());


	MatrixTypeDoublePrecision Minv = M.cast<double>().inverse();

	// the MAP solution for the latent variables (coefficients)
	VectorType coeffs = Minv.cast<ScalarType>() * WT * (samplePart - muPart);

	// the MAP solution in the sample space
	VectorType newMean = inputModel->GetRepresenter()->SampleToSampleVector(inputModel->DrawSample(coeffs));

	// We note that the posterior distribution can again be seen as  PPCA model
	// (i.e. any sample S  can be written in the form S =  mu + W alpha + epsilon)
	// To obtain the matrix W for this posterior model, we need to perform a pca of
	// the posterior covariance matrix given by
	// pcaBasisMatrix * variance * Minv * pcaBasisMatrix^T.
	//
	// We could decompose this matrix using an SVD, as we do in the PCAModelBuilder.
	// However, there is a more efficient way. We can use the fact that the PCABasisMatrix
	// of the input model is composed of an orthonormal matrix U (the eigenvectors fo the data covariance)
	// and the pcaSdev D (e.g. W = UD). We have that the covariance is given by
	// U * pcaVariance^{1/2} * Minv * pcaVariance^{1/2} * U.T
	// We see that all the variance terms are in the inner part of above expression (without the U).
	// We can now compute an SVD of this inner part, say
	// pcaVariance^{1/2} * Minv * pcaVariance^{1/2} = Uhat Dhat^2 Uhat^T.
	// We then take U * Uhat as the new pcaBasis and Dhat^2 is the new variance. Together with the mean, this defines
	// again a new, valid ppca model.
	// If U is orthonormal, then we see that U*Uhat actually diagonalizes the matrix. So we even get back the classic
	// interpretation of the U as the eigenvectors of the (now constrained) covariance matrix.
	//
	// For a more detailed explanation, consult the paper
	//  Probabilistic Modeling and Visualization of the	Flexibility in Morphable Models, M. Luethi, T.Albrecht, T.Vetter
	// and the book C. Bishop, PRML Chapter 12
	//
	const VectorType& pcaVariance = inputModel->GetPCAVarianceVector();
	VectorTypeDoublePrecision pcaSdev = pcaVariance.cast<double>().array().sqrt();

	typedef Eigen::JacobiSVD<MatrixTypeDoublePrecision> SVDType;
	MatrixTypeDoublePrecision innerMatrix = pcaSdev.asDiagonal() * Minv * pointValuesNoiseVariance * pcaSdev.asDiagonal();
	SVDType svd(innerMatrix, Eigen::ComputeThinU);


	VectorType newPCAVariance = svd.singularValues().cast<ScalarType>();

	MatrixType newPCABasisMatrix = pcaBasisMatrix * svd.matrixU().cast<ScalarType>();

	StatisticalModelType* partiallyFixedModel = StatisticalModelType::Create(representer,newMean, newPCABasisMatrix, newPCAVariance, noiseVariance);

	// Write the parameters used to build the models into the builderInfo

	typename ModelInfo::BuilderInfoList builderInfoList = inputModel->GetModelInfo().GetBuilderInfoList();

	BuilderInfo::ParameterInfoList bi;
	bi.push_back(BuilderInfo::KeyValuePair("NoiseVariance ", Utils::toString(noiseVariance)));
	bi.push_back(BuilderInfo::KeyValuePair("FixedPointsVariance ", Utils::toString(pointValuesNoiseVariance)));
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


	BuilderInfo builderInfo("PartiallyFixedModelBuilder", di, bi);
	builderInfoList.push_back(builderInfo);

	MatrixType inputScores = inputModel->GetModelInfo().GetScoresMatrix();
	MatrixType scores = MatrixType::Zero(inputScores.rows(), inputScores.cols());

	if (computeScores == true) {

		// get the scores from the input model
		for (unsigned i = 0; i < inputScores.cols(); i++) {
			// reconstruct the sample from the input model and project it back into the model
			typename Representer::DatasetPointerType ds = inputModel->DrawSample(inputScores.col(i));
			scores.col(i) = partiallyFixedModel->ComputeCoefficientsForDataset(ds);
			Representer::DeleteDataset(ds);
		}
	}
	ModelInfo info(scores, builderInfoList);
	partiallyFixedModel->SetModelInfo(info);

	return partiallyFixedModel;

}

} // namespace statismo
