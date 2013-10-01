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


#include "statismo/PosteriorModelBuilder.h"
#include "statismo/StatisticalModel.h"
#include "statismo/DataManager.h"

#include "Representers/VTK/vtkStandardMeshRepresenter.h"

#include "vtkPolyData.h"

#include <iostream>
#include <memory>

using namespace statismo;
using std::auto_ptr;


//
// This example shows how a statistical shape model can be built, when the position of some of the points
// is known and fixed. // The constraint model can either be built from data sets that are added to the DataManager (see e.g.
// BuildShapeModelExample.cxx) or from an already built model. We use the second option here.
// In this example we simply fix the first point in the domain to the mean value of our input model.
//
int main(int argc, char** argv) {

	if (argc < 3) {
		std::cout << "Usage " << argv[0] << " inputModel constraintModel" << std::endl;
		exit(-1);
	}
	std::string inputModelName(argv[1]);
	std::string constraintModelName(argv[2]);



	// All the statismo classes have to be parameterized with the RepresenterType.
	// For building a shape model with vtk, we use the vtkPolyDataRepresenter.

	typedef vtkStandardMeshRepresenter RepresenterType;
	typedef StatisticalModel<vtkPolyData> StatisticalModelType;
	typedef PosteriorModelBuilder<vtkPolyData> PosteriorModelBuilderType;

	typedef StatisticalModelType::DomainType DomainType;

	try {
		// load the model
		RepresenterType* representer = RepresenterType::Create();
		auto_ptr<StatisticalModelType> inputModel(StatisticalModelType::Load(representer, inputModelName));


		auto_ptr<PosteriorModelBuilderType> pfmb(PosteriorModelBuilderType::Create());

		// For simplicity, we simply fix the 1st point in the domain
		const DomainType::DomainPointsListType& domainPoints = inputModel->GetDomain().GetDomainPoints();


		vtkPoint fixedPoint = domainPoints.front();
		vtkPoint meanValueForPt = inputModel->DrawMeanAtPoint(fixedPoint);

		// Create an empty list, holding the constraint and add the point
		StatisticalModelType::PointValueListType constraints;

		StatisticalModelType::PointValuePairType pointValue(fixedPoint, meanValueForPt);
		constraints.push_back(pointValue);

		// build the new model. In addition to the input model and the constraints, we also specify
		// the inaccuracy of our value (variance of the error).
		auto_ptr<StatisticalModelType> constraintModel(pfmb->BuildNewModelFromModel(inputModel.get(), constraints, 0.1));

		// The resulting model is a normal statistical model, from which we could for example sample examples.
		// Here we simply  save it to disk for later use.
		constraintModel->Save(constraintModelName);
		std::cout << "successfully saved the model to " << constraintModelName << std::endl;
	}
	catch (StatisticalModelException& e) {
		std::cout << "Exception occured while building the intenisity model" << std::endl;
		std::cout << e.what() << std::endl;
	}
}
