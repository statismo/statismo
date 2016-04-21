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
#include <string>

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>

#include <itkImage.h>
#include <itkMesh.h>
#include <itkReducedVarianceModelBuilder.h>
#include <itkStandardImageRepresenter.h>
#include <itkStandardMeshRepresenter.h>
#include <itkStatismoIO.h>
#include <itkStatisticalModel.h>


namespace po = boost::program_options;
using namespace std;

struct programOptions {
    bool bDisplayHelp;
    string strInputFileName;
    string strOutputFileName;
    unsigned uNumberOfComponents;
    unsigned uNumberOfDimensions;
    double dTotalVariance;
    string strType;
};

po::options_description initializeProgramOptions(programOptions& poParameters);
bool isOptionsConflictPresent(programOptions& opt);
template <class DataType, class RepresenterType>
void reduceModel(programOptions opt);



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
        const unsigned Dimensionality2D = 2;
        const unsigned Dimensionality3D = 3;
        if(poParameters.strType == "shape") {
            typedef itk::Mesh<float, Dimensionality3D> DataType;
            typedef itk::StandardMeshRepresenter<float, Dimensionality3D> RepresenterType;
            reduceModel<DataType, RepresenterType>(poParameters);
        } else {
            if (poParameters.uNumberOfDimensions == Dimensionality2D) {
                typedef itk::Vector<float, Dimensionality2D> VectorPixelType;
                typedef itk::Image<VectorPixelType, Dimensionality2D> DataType;
                typedef itk::StandardImageRepresenter<VectorPixelType, Dimensionality2D> RepresenterType;
                reduceModel<DataType, RepresenterType>(poParameters);
            } else {
                typedef itk::Vector<float, Dimensionality3D> VectorPixelType;
                typedef itk::Image<VectorPixelType, Dimensionality3D> DataType;
                typedef itk::StandardImageRepresenter<VectorPixelType, Dimensionality3D> RepresenterType;
                reduceModel<DataType, RepresenterType>(poParameters);
            }
        }
    } catch (itk::ExceptionObject & e) {
        cerr << "Could not reduce the model:" << endl;
        cerr << e.what() << endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

bool isOptionsConflictPresent(programOptions& opt) {
    boost::algorithm::to_lower(opt.strType);

    if (opt.strType != "shape" && opt.strType != "deformation") {
        return true;
    }

    if (opt.dTotalVariance != 0 && opt.uNumberOfComponents != 0) {
        return true;
    }

    if (opt.dTotalVariance == 0 && opt.uNumberOfComponents == 0) {
        return true;
    }

    if (opt.dTotalVariance > 100) {
        return true;
    }

    if (opt.strInputFileName == "" || opt.strOutputFileName == "") {
        return true;
    }


    if (opt.strInputFileName == opt.strOutputFileName) {
        return true;
    }

    return false;
}


template <class DataType, class RepresenterType>
void reduceModel(programOptions opt) {
    typename RepresenterType::Pointer pRepresenter = RepresenterType::New();


    typedef typename itk::StatisticalModel<DataType> StatisticalModelType;
    typename StatisticalModelType::Pointer pModel = StatisticalModelType::New();

    pModel = itk::StatismoIO<DataType>::LoadStatisticalModel(pRepresenter, opt.strInputFileName.c_str());

    typedef typename itk::ReducedVarianceModelBuilder<DataType> ReducedVarianceModelBuilderType;
    typename ReducedVarianceModelBuilderType::Pointer pReducedVarModelBuilder = ReducedVarianceModelBuilderType::New();
    typename StatisticalModelType::Pointer pOutputModel;

    if (opt.uNumberOfComponents != 0) {
        if (opt.uNumberOfComponents > pModel->GetNumberOfPrincipalComponents()) {
            itkGenericExceptionMacro(<< "The model has " << pModel->GetNumberOfPrincipalComponents() << " components. Upscaling models to more componenets isn't possible with this tool.");
        }
        pOutputModel = pReducedVarModelBuilder->BuildNewModelWithLeadingComponents(pModel.GetPointer(), opt.uNumberOfComponents);
    } else {
        pOutputModel = pReducedVarModelBuilder->BuildNewModelWithVariance(pModel.GetPointer(), opt.dTotalVariance);
    }

    itk::StatismoIO<DataType>::SaveStatisticalModel(pOutputModel, opt.strOutputFileName.c_str());
}


po::options_description initializeProgramOptions(programOptions& poParameters) {
    po::options_description optMandatory("Mandatory options");
    optMandatory.add_options()
    ("type,t", po::value<string>(&poParameters.strType)->default_value("shape"), "Specifies the type of the model: shape and deformation are the two available types")
    ("dimensionality,d", po::value<unsigned>(&poParameters.uNumberOfDimensions)->default_value(3), "Dimensionality of the input model (only available if the type is deformation)")
    ("input-file,i", po::value<string>(&poParameters.strInputFileName), "The path to the model file.")
    ("output-file,o", po::value<string>(&poParameters.strOutputFileName), "Name of the output file where the reduced model will be written to.")
    ;
    po::options_description optAdditional("Optional options");
    optAdditional.add_options()
    ("totalvariance,v", po::value<double>(&poParameters.dTotalVariance)->default_value(0), "Creates a new Model that will have a fraction of the old models' variance. This parameter is in percent and thus ranges from 0 to 100.")
    ("numcomponents,n", po::value<unsigned>(&poParameters.uNumberOfComponents)->default_value(0), "Creates a new model with the specified number of components.")
    ("help,h", po::bool_switch(&poParameters.bDisplayHelp), "Display this help message")
    ;

    po::options_description optAllOptions;
    optAllOptions.add(optMandatory).add(optAdditional);
    return optAllOptions;
}
