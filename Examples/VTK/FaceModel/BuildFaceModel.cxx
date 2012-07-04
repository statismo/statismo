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



/*
 * This example builds a face model from 3D scans of a face.
 * To run this example, you need to download the example data from the website
 */

#include "statismo/PCAModelBuilder.h"
#include "statismo/StatisticalModel.h"
#include "statismo/DataManager.h"

#include "Representers/VTK/vtkPolyDataRepresenter.h"

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


void saveSample(const vtkPolyData* pd, const std::string& resdir, const std::string& basename)
{
	std::string filename = resdir +std::string("/") + basename;

	vtkPolyDataWriter* w = vtkPolyDataWriter::New();
	w->SetInput(const_cast<vtkPolyData*>(pd));
	w->SetFileName(filename.c_str());
	w->Update();
}

//
// Build a new shape model from a set of faces
// We then generate a number of samples into the resultdir
//
int main(int argc, char** argv) {

	if (argc < 4) {
		std::cout << "Usage " << argv[0] << " facedir numSamples resultdir" << std::endl;
		std::cout << "Note that you need to download the example data to be able to run this example" << std::endl;
		exit(-1);
	}
	std::string datadir(argv[1]);
	int numSamples(atoi(argv[2]));
	std::string resultdir(argv[3]);

	typedef vtkPolyDataRepresenter RepresenterType;
	typedef DataManager<RepresenterType> DataManagerType;
	typedef PCAModelBuilder<RepresenterType> ModelBuilderType;
	typedef StatisticalModel<RepresenterType> StatisticalModelType;

	try {

		vtkPolyData* reference = loadVTKPolyData(datadir +"/face-1.vtk");
		auto_ptr<RepresenterType> representer(RepresenterType::Create(reference, RepresenterType::RIGID));

		auto_ptr<DataManagerType> dataManager(DataManagerType::Create(representer.get()));

		for (unsigned i = 1; i <= 11; i++) {

			std::ostringstream ss;
			ss << datadir +"/face-" << i << ".vtk";
			const std::string datasetFilename = ss.str();
			vtkPolyData* dataset = loadVTKPolyData(datasetFilename);
			dataManager->AddDataset(dataset, datasetFilename);
			dataset->Delete();
		}

		auto_ptr<ModelBuilderType> modelBuilder(ModelBuilderType::Create());
		auto_ptr<StatisticalModelType> model(modelBuilder->BuildNewModel(dataManager->GetSampleData(), 0.01));

		model->Save(resultdir +"/facemodel.h5");

		// draw a number of samples, to visualize the variability in the model
		for (unsigned i = 0; i < numSamples; i++) {
			vtkPolyData* sample = model->DrawSample();
			std::ostringstream ss;
			ss << "facesample-" << i << ".vtk";
			saveSample(sample, resultdir, ss.str());
		}

		reference->Delete();
	}
	catch (StatisticalModelException& e) {
		std::cout << "Exception occured while building the shape model" << std::endl;
		std::cout << e.what() << std::endl;
	}
}
