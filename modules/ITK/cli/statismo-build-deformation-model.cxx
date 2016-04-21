/*
 * Copyright (c) 2015 University of Basel
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

#include <boost/program_options.hpp>

#include <itkDataManager.h>
#include <itkDirectory.h>
#include <itkImageFileReader.h>
#include <itkPCAModelBuilder.h>
#include <itkStandardImageRepresenter.h>
#include <itkStatismoIO.h>
#include <itkStatisticalModel.h>

#include "utils/statismo-build-models-utils.h"

namespace po = boost::program_options;
using namespace std;

struct programOptions {
    bool bDisplayHelp;
    bool bComputeScores;
    string strDataListFile;
    string strOutputFileName;
    float fNoiseVariance;
    unsigned uNumberOfDimensions;
};

po::options_description initializeProgramOptions(programOptions& poParameters);
bool isOptionsConflictPresent(programOptions& opt);
template<unsigned Dimensions>
void buildAndSaveDeformationModel(programOptions opt);



int main(int argc, char** argv) {
    programOptions poParameters;

    po::positional_options_description optPositional;
    optPositional.add("output-file", 1);
    po::options_description optAllOptions = initializeProgramOptions(poParameters);


    po::variables_map vm;
    try {
        po::parsed_options parsedOptions = po::command_line_parser(argc, argv).options(optAllOptions).positional(optPositional).run();
        po::store(parsedOptions, vm);
        po::notify(vm);
    } catch (po::error& e) {
        cerr << "An exception occurred while parsing the Command line:"<<endl;
        cerr << e.what() << endl;
        return EXIT_FAILURE;
    }

    if (poParameters.bDisplayHelp == true) {
        cout << optAllOptions << endl;
        return EXIT_SUCCESS;
    }
    if (isOptionsConflictPresent(poParameters) == true)	{
        cerr << "A conflict in the options exists or insufficient options were set." << endl;
        cout << optAllOptions << endl;
        return EXIT_FAILURE;
    }

    try {
        if (poParameters.uNumberOfDimensions == 2) {
            buildAndSaveDeformationModel<2>(poParameters);
        } else {
            buildAndSaveDeformationModel<3>(poParameters);
        }

    } catch (ifstream::failure & e) {
        cerr << "Could not read the data-list:" << endl;
        cerr << e.what() << endl;
        return EXIT_FAILURE;
    } catch (itk::ExceptionObject & e) {
        cerr << "Could not build the model:" << endl;
        cerr << e.what() << endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

bool isOptionsConflictPresent(programOptions& opt) {
    if (opt.strDataListFile == "" || opt.strOutputFileName == "" ) {
        return true;
    }

    if (opt.strDataListFile == opt.strOutputFileName) {
        return true;
    }

    if (opt.fNoiseVariance < 0) {
        return true;
    }

    if (opt.uNumberOfDimensions != 2 && opt.uNumberOfDimensions != 3) {
        return true;
    }

    return false;
}

template<unsigned Dimensions>
void buildAndSaveDeformationModel(programOptions opt) {
    typedef itk::Vector<float, Dimensions> VectorPixelType;
    typedef itk::Image<VectorPixelType, Dimensions> ImageType;
    typedef itk::StandardImageRepresenter<VectorPixelType, Dimensions > RepresenterType;
    typename RepresenterType::Pointer representer = RepresenterType::New();

    typedef itk::DataManager<ImageType> DataManagerType;
    typename DataManagerType::Pointer dataManager = DataManagerType::New();

    StringList fileNames = getFileList(opt.strDataListFile);

    typedef itk::ImageFileReader<ImageType> ImageReaderType;
    typedef vector<typename ImageReaderType::Pointer> ImageReaderList;

    if (fileNames.size() == 0) {
        itkGenericExceptionMacro( << "No Data was loaded and thus the model can't be built.");
    }
    bool firstPass = true;
    for (StringList::const_iterator it = fileNames.begin(); it != fileNames.end(); ++it) {

        typename ImageReaderType::Pointer reader = ImageReaderType::New();

        reader->SetFileName(it->c_str());
        reader->Update();
        if ( firstPass ) {
            representer->SetReference(reader->GetOutput());
            dataManager->SetRepresenter(representer);
            firstPass = false;
        }
        dataManager->AddDataset(reader->GetOutput(), reader->GetFileName().c_str());
    }

    typedef itk::StatisticalModel<ImageType> StatisticalModelType;
    typename StatisticalModelType::Pointer model;

    typedef itk::PCAModelBuilder<ImageType> ModelBuilderType;
    typename ModelBuilderType::Pointer pcaModelBuilder = ModelBuilderType::New();
    model = pcaModelBuilder->BuildNewModel(dataManager->GetData(), opt.fNoiseVariance, opt.bComputeScores);
    itk::StatismoIO<ImageType>::SaveStatisticalModel(model, opt.strOutputFileName.c_str());
}

po::options_description initializeProgramOptions(programOptions& poParameters) {
    po::options_description optMandatory("Mandatory options");
    optMandatory.add_options()
    ("data-list,l", po::value<string>(&poParameters.strDataListFile), "File containing a list of meshes to build the deformation model from")
    ("output-file,o", po::value<string>(&poParameters.strOutputFileName), "Name of the output file")
    ("dimensionality,d", po::value<unsigned>(&poParameters.uNumberOfDimensions)->default_value(3), "Dimensionality of the input images in the data list")
    ;
    po::options_description optAdditional("Optional options");
    optAdditional.add_options()
    ("noise,n", po::value<float>(&poParameters.fNoiseVariance)->default_value(0), "Noise variance of the PPCA model")
    ("scores,s", po::value<bool>(&poParameters.bComputeScores)->default_value(true), "Compute scores (default true)")
    ("help,h", po::bool_switch(&poParameters.bDisplayHelp), "Display this help message")
    ;

    po::options_description optAllOptions;
    optAllOptions.add(optMandatory).add(optAdditional);
    return optAllOptions;
}
