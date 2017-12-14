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

#include <iostream>
#include <set>
#include <string>

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>

#include <itkDataManager.h>
#include <itkDirectory.h>
#include <itkImage.h>
#include <itkMesh.h>
#include <itkMeshFileReader.h>
#include <itkMeshFileWriter.h>
#include <itkStandardImageRepresenter.h>
#include <itkStandardMeshRepresenter.h>
#include <itkStatismoIO.h>
#include <itkStatisticalModel.h>


const unsigned Dimensionality3D = 3;
const unsigned Dimensionality2D = 2;

namespace po = boost::program_options;
using namespace std;

typedef vector<string> StringList;

struct programOptions {
    bool bDisplayHelp;
    string strInputFileName;
    string strOutputFileName;
    string strType;
    StringList vParameters;
    bool bSampleMean;
    bool bSampleReference;
    unsigned uNumberOfDimensions;
};

po::options_description initializeProgramOptions(programOptions& poParameters);
bool isOptionsConflictPresent(programOptions& opt);
template <class DataType, class RepresenterType, class DataWriterType>
void drawSampleFromModel(programOptions opt);
template <class VectorType>
void populateVectorWithParameters(const StringList& vParams, VectorType& vParametersReturnVector);



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
        if (poParameters.strType == "shape") {
            typedef itk::Mesh<float, Dimensionality3D> DataType;
            typedef itk::StandardMeshRepresenter<float, Dimensionality3D> RepresenterType;
            typedef itk::MeshFileWriter<DataType> DataWriterType;
            drawSampleFromModel<DataType, RepresenterType, DataWriterType>(poParameters);
        } else {
            if (poParameters.uNumberOfDimensions == 2) {
                typedef itk::Vector<float, Dimensionality2D> VectorPixelType;
                typedef itk::Image<VectorPixelType, Dimensionality2D> DataType;
                typedef itk::StandardImageRepresenter<VectorPixelType, Dimensionality2D> RepresenterType;
                typedef itk::ImageFileWriter<DataType> DataWriterType;
                drawSampleFromModel<DataType, RepresenterType, DataWriterType>(poParameters);
            } else {
                typedef itk::Vector<float, Dimensionality3D> VectorPixelType;
                typedef itk::Image<VectorPixelType, Dimensionality3D> DataType;
                typedef itk::StandardImageRepresenter<VectorPixelType, Dimensionality3D> RepresenterType;
                typedef itk::ImageFileWriter<DataType> DataWriterType;
                drawSampleFromModel<DataType, RepresenterType, DataWriterType>(poParameters);
            }
        }
    } catch (itk::ExceptionObject & e) {
        cerr << "Could not get a sample:" << endl;
        cerr << e.what() << endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

bool isOptionsConflictPresent(programOptions& opt) {
    boost::algorithm::to_lower(opt.strType);

    if (opt.bSampleMean + (opt.vParameters.size() > 0) + opt.bSampleReference > 1) {
        return true;
    }

    if (opt.strInputFileName == "" || opt.strOutputFileName == "") {
        return true;
    }

    if (opt.strType != "shape" && opt.strType != "deformation") {
        return true;
    }

    if (opt.strInputFileName == opt.strOutputFileName) {
        return true;
    }

    return false;
}


template <class VectorType>
void populateVectorWithParameters(const StringList& vParams, VectorType& vParametersReturnVector) {
    set<unsigned> sSeenIndices;

    for (StringList::const_iterator i = vParams.begin(); i != vParams.end(); ++i) {
        StringList vSplit;
        bool bSuccess = true;
        boost::split(vSplit, *i, boost::is_any_of(":"));
        if (vSplit.size() != 2) {
            bSuccess = false;
        } else {
            try {
                unsigned uIndex = boost::lexical_cast<unsigned>(vSplit[0])-1;
                double dValue = boost::lexical_cast<double>(vSplit[1]);

                if (uIndex >= vParametersReturnVector.size()) {
                    itkGenericExceptionMacro( << "The parameter '" << *i << "' is has an index value that is not in the range of this model's available parameters (1 to " <<vParametersReturnVector.size() << ").");
                }

                if (sSeenIndices.find(uIndex) == sSeenIndices.end()) {
                    sSeenIndices.insert(uIndex);
                    vParametersReturnVector[uIndex] = dValue;
                } else {
                    itkGenericExceptionMacro( << "The index '" << (uIndex+1)<<"' occurs more than once in the parameter list.");
                }
            } catch (boost::bad_lexical_cast &) {
                bSuccess = false;
            }
        }

        if (bSuccess == false) {
            itkGenericExceptionMacro( << "The parameter '" << *i << "' is in an incorrect format. The correct format is index:value. Like for example 0:1.1 or 19:-2");
        }
    }
}

template <class DataType, class RepresenterType, class DataWriterType>
void drawSampleFromModel(programOptions opt) {
    typename RepresenterType::Pointer pRepresenter = RepresenterType::New();

    typedef itk::StatisticalModel<DataType> StatisticalModelType;
    typename StatisticalModelType::Pointer pModel = StatisticalModelType::New();

    pModel = itk::StatismoIO<DataType>::LoadStatisticalModel(pRepresenter, opt.strInputFileName.c_str());

    typename DataType::Pointer output;
    if (opt.bSampleMean == true) {
        output = pModel->DrawMean();
    } else if (opt.vParameters.size() > 0) {
        unsigned uNrOfParameters = pModel->GetNumberOfPrincipalComponents();
        typename StatisticalModelType::VectorType vModelParameters(uNrOfParameters);
        vModelParameters.fill(0);

        populateVectorWithParameters<typename StatisticalModelType::VectorType>(opt.vParameters, vModelParameters);

        output = pModel->DrawSample(vModelParameters);
    } else if (opt.bSampleReference == true) {
        output = pModel->GetRepresenter()->GetReference();
    } else {
        output = pModel->DrawSample();
    }

    typename DataWriterType::Pointer pDataWriter = DataWriterType::New();
    pDataWriter->SetFileName(opt.strOutputFileName.c_str());
    pDataWriter->SetInput(output);
    pDataWriter->Update();
}


po::options_description initializeProgramOptions(programOptions& poParameters) {
    po::options_description optMandatory("Mandatory options");
    optMandatory.add_options()
    ("type,t", po::value<string>(&poParameters.strType)->default_value("shape"), "Specifies the type of the model: shape and deformation are the two available types")
    ("dimensionality,d", po::value<unsigned>(&poParameters.uNumberOfDimensions)->default_value(3), "Dimensionality of the input model (only available if the type is deformation)")
    ("input-file,i", po::value<string>(&poParameters.strInputFileName), "The path to the model file.")
    ("output-file,o", po::value<string>(&poParameters.strOutputFileName), "Name of the output file/the sample.")
    ;
    po::options_description optAdditional("Optional options");
    optAdditional.add_options()
    ("mean,m", po::bool_switch(&poParameters.bSampleMean), "Draws the mean from the model and saves it.")
    ("reference,r", po::bool_switch(&poParameters.bSampleReference), "Draws the reference from the model and saves it.")
    ("parameters,p", po::value<StringList >(&poParameters.vParameters)->multitoken(), "Makes it possible to specify a list of parameters and their positions that will then be used to draw a sample. Parameters are speciefied in the following format: POSITION1:VALUE1 POSITIONn:VALUEn. Unspecified parameters will be set to 0. The first parameter is at position 1.")
    ("help,h", po::bool_switch(&poParameters.bDisplayHelp), "Display this help message")
    ;

    po::options_description optAllOptions;
    optAllOptions.add(optMandatory).add(optAdditional);
    return optAllOptions;
}
