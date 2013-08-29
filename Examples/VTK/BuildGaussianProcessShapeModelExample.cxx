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


#include "statismo/LowRankGPModelBuilder.h"
#include "statismo/StatisticalModel.h"
#include "vtkPolyDataReader.h"
#include "vtkStandardMeshRepresenter.h"

#include <iostream>
#include <memory>

using namespace statismo;
using std::auto_ptr;

vtkPolyData* loadVTKPolyData(const std::string& filename)
{
	vtkPolyDataReader* reader = vtkPolyDataReader::New();
	reader->SetFileName(filename.c_str());
	reader->Update();
	vtkPolyData* pd = vtkPolyData::New();
	pd->ShallowCopy(reader->GetOutput());
	return pd;
}






//
// Build a new shape model TODO document me
//
int main(int argc, char** argv) {

	if (argc < 5) {
		std::cout << "Usage " << argv[0] << " reference gaussianKernelWidth numberOfComponents, modelname" << std::endl;
		exit(-1);
	}
	std::string referenceFilename(argv[1]);
	double gaussianKernelSigma = std::atof(argv[2]);
	int numberOfComponents = std::atoi(argv[3]);
	std::string modelname(argv[4]);


	// All the statismo classes have to be parameterized with the RepresenterType.
	// For building a shape model with vtk, we use the vtkPolyDataRepresenter.
	typedef vtkStandardMeshRepresenter RepresenterType;
	typedef LowRankGPModelBuilder<vtkPolyData> ModelBuilderType;
	typedef StatisticalModel<vtkPolyData> StatisticalModelType;

	try {

		// We create a new representer object. For the vtkPolyDataRepresenter, we have to set a reference.
		vtkPolyData* reference = loadVTKPolyData(referenceFilename);
		auto_ptr<RepresenterType> representer(RepresenterType::Create(reference));

		// create a scalar valued kernel
		const GaussianKernel gk = GaussianKernel(gaussianKernelSigma);

		// make the kernel matrix valued and scale it by a factor of 100
		const MatrixValuedKernel<3>& mvGk = UncorrelatedMatrixValuedKernel<3>(&gk);
		const MatrixValuedKernel<3>& scaledGk = ScaledKernel<3>(&mvGk, 100.0);


		auto_ptr<ModelBuilderType> modelBuilder(ModelBuilderType::Create(representer.get()));
		auto_ptr<StatisticalModelType> model(modelBuilder->BuildNewModel(reference, scaledGk, numberOfComponents));

		// Once we have built the model, we can save it to disk.
		model->Save(modelname);
		std::cout << "Successfully saved shape model as " << modelname << std::endl;

		reference->Delete();
	}
	catch (StatisticalModelException& e) {
		std::cout << "Exception occured while building the shape model" << std::endl;
		std::cout << e.what() << std::endl;
	}
}
