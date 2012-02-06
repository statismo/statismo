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


#include "statismo/PCAModelBuilder.h"
#include "statismo/StatisticalModel.h"
#include "statismo/DataManager.h"

#include "Representers/VTK/vtkStructuredPointsRepresenter.h"

#include <iostream>
#include <memory>

using namespace statismo;
using std::auto_ptr;


//
// Build a new shape model from vtkPolyData, given in datadir.
//
int main(int argc, char** argv) {

	if (argc < 3) {
		std::cout << "Usage " << argv[0] << " datadir modelname" << std::endl;
		exit(-1);
	}
	std::string datadir(argv[1]);
	std::string modelname(argv[2]);


	// All the statismo classes have to be parameterized with the RepresenterType.
	// For building a intensity model with vtk, we use the vtkStructuredPointsRepresenter.
	// Here, we work with unsigned character images. The second template parameter specifies
	// the pixel dimension (1 means scalar image, whereas 3 is a 3D vector image).
	typedef vtkStructuredPointsRepresenter<unsigned char, 1> RepresenterType;
	typedef DataManager<RepresenterType> DataManagerType;
	typedef PCAModelBuilder<RepresenterType> ModelBuilderType;
	typedef StatisticalModel<RepresenterType> StatisticalModelType;

	try {

		// Model building is exactly the same as for shape models (see BuildShapeModelExample for detailed explanation)
		auto_ptr<RepresenterType> representer(RepresenterType::Create(datadir +"/hand-0.vtk"));
		auto_ptr<DataManagerType> dataManager(DataManagerType::Create(representer.get()));

		dataManager->AddDataset(datadir +"/hand-0.vtk");
		dataManager->AddDataset(datadir +"/hand-1.vtk");
		dataManager->AddDataset(datadir +"/hand-2.vtk");
		dataManager->AddDataset(datadir +"/hand-3.vtk");

		auto_ptr<ModelBuilderType> modelBuilder(ModelBuilderType::Create());
		auto_ptr<StatisticalModelType> model(modelBuilder->BuildNewModel(dataManager->GetSampleData(), 0.01));
		model->Save(modelname);
		std::cout << "Succesfully saved model as " << modelname << std::endl;
	}
	catch (StatisticalModelException& e) {
		std::cout << "Exception occured while building the intenisity model" << std::endl;
		std::cout << e.what() << std::endl;
	}
}
