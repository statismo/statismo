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
#include "Exceptions.h"
#include <iostream>



namespace statismo {




template <typename Representer>
ReducedVarianceModelBuilder<Representer>::ReducedVarianceModelBuilder()
: Superclass()
  {}


template <typename Representer>
typename ReducedVarianceModelBuilder<Representer>::StatisticalModelType*
ReducedVarianceModelBuilder<Representer>::BuildNewModelFromModel(
		const StatisticalModelType* inputModel,
		double totalVariance,
		bool computeScores) const
{

	VectorType pcaVariance = inputModel->GetPCAVarianceVector();
	  double modelVariance = pcaVariance.sum();

	  //count the number of modes required for the model
	  double cumulatedVariance = 0;
	  unsigned numComponentsToReachPrescribedVariance = 0;
	  for (unsigned i = 0; i < pcaVariance.size(); i++) {
		  cumulatedVariance += pcaVariance(i);
		  if (cumulatedVariance / modelVariance >= totalVariance)
			  break;
		  numComponentsToReachPrescribedVariance++;
	  }

	  StatisticalModelType* reducedModel = StatisticalModelType::Create(
			  inputModel->GetRepresenter(),
			  inputModel->GetMeanVector(),
			  inputModel->GetOrthonormalPCABasisMatrix().leftCols(numComponentsToReachPrescribedVariance),
			  inputModel->GetPCAVarianceVector().topRows(numComponentsToReachPrescribedVariance),
			  inputModel->GetNoiseVariance());

	// Write the parameters used to build the models into the builderInfo
	typename ModelInfo::BuilderInfoList builderInfoList = inputModel->GetModelInfo().GetBuilderInfoList();

	BuilderInfo::ParameterInfoList bi;
	bi.push_back(BuilderInfo::KeyValuePair("totalVariance ", Utils::toString(totalVariance)));

	BuilderInfo::DataInfoList di;


	BuilderInfo builderInfo("ReducedVarianceModelBuilder", di, bi);
	builderInfoList.push_back(builderInfo);

	ModelInfo info(inputModel->GetModelInfo().GetScoresMatrix().topRows(numComponentsToReachPrescribedVariance), builderInfoList);
	reducedModel->SetModelInfo(info);

	return reducedModel;

}


} // namespace statismo
