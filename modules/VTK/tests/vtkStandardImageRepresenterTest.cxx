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

#include "genericRepresenterTest.hxx"
#include "vtkStandardImageRepresenter.h"


typedef statismo::vtkStandardImageRepresenter<double, 2> RepresenterType;
typedef GenericRepresenterTest< RepresenterType> RepresenterTestType;

vtkStructuredPoints* loadStructuredPoints(const std::string& filename) {
    vtkStructuredPointsReader* reader = vtkStructuredPointsReader::New();
    reader->SetFileName(filename.c_str());
    reader->Update();
    vtkStructuredPoints* pd = vtkStructuredPoints::New();
    pd->ShallowCopy(reader->GetOutput());
    reader->Delete();
    return pd;
}

int main(int argc, char** argv) {

    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " datadir" << std::endl;
        exit(EXIT_FAILURE);
    }
    std::string datadir = std::string(argv[1]);

    const std::string referenceFilename = datadir + "/hand_dfs/df-hand-1.vtk";
    const std::string testDatasetFilename = datadir + "/hand_dfs/df-hand-2.vtk";

    vtkStructuredPoints* reference = loadStructuredPoints(referenceFilename);

    RepresenterType* representer = RepresenterType::Create(reference);

    // choose a test dataset, a point (on the reference) and the associated point on the test example
    vtkStructuredPoints* testDataset = loadStructuredPoints(testDatasetFilename);
    unsigned testPtId = 0;
    statismo::vtkPoint testPt(reference->GetPoint(testPtId));
    statismo::vtkNDPixel testValue(testDataset->GetPointData()->GetScalars()->GetTuple2(testPtId), 2);
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


