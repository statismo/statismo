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

#include <vtkPolyDataReader.h>
#include <vtkSmartPointer.h>

#include "genericRepresenterTest.hxx"
#include "vtkStandardMeshRepresenter.h"

using statismo::vtkStandardMeshRepresenter;
using statismo::vtkPoint;

typedef GenericRepresenterTest<vtkStandardMeshRepresenter> RepresenterTestType;

vtkPolyData* loadPolyData(const std::string& filename) {
    vtkSmartPointer<vtkPolyDataReader> reader = vtkSmartPointer<vtkPolyDataReader>::New();
    reader->SetFileName(filename.c_str());
    reader->Update();
    vtkPolyData* pd = vtkPolyData::New();
    pd->ShallowCopy(reader->GetOutput());
    return pd;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " datadir" << std::endl;
        exit(EXIT_FAILURE);
    }
    std::string datadir = std::string(argv[1]);

    const std::string referenceFilename = datadir + "/hand_polydata/hand-0.vtk";
    const std::string testDatasetFilename = datadir + "/hand_polydata/hand-1.vtk";

    vtkPolyData* reference = loadPolyData(referenceFilename);
    vtkStandardMeshRepresenter* representer = vtkStandardMeshRepresenter::Create(reference);

    // choose a test dataset, a point (on the reference) and the associated point on the test example

    vtkPolyData* testDataset = loadPolyData(testDatasetFilename);
    unsigned testPtId = 0;
    vtkPoint testPt(reference->GetPoints()->GetPoint(testPtId));
    vtkPoint testValue(testDataset->GetPoints()->GetPoint(testPtId));

    RepresenterTestType representerTest(representer, testDataset, std::make_pair(testPt, testValue));

    bool testsOk = representerTest.runAllTests();
    delete representer;
    reference->Delete();
    testDataset->Delete();

    if (testsOk == true) {
        return EXIT_SUCCESS;
    } else {
        return EXIT_FAILURE;
    }

}


