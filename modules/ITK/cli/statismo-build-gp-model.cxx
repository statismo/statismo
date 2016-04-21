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

#include <boost/program_options.hpp>

#include <itkDataManager.h>
#include <itkDirectory.h>
#include <itkImageFileReader.h>
#include <itkLowRankGPModelBuilder.h>
#include <itkMeshFileReader.h>
#include <itkStandardImageRepresenter.h>
#include <itkStandardMeshRepresenter.h>
#include <itkStatismoIO.h>
#include <itkStatisticalModel.h>

//Add new kernels in this file (and document their usage in the statismo-build-gp-model.md file)
#include "utils/statismo-build-gp-model-kernels.h"

namespace po = boost::program_options;
using namespace std;

struct programOptions {
    bool bDisplayHelp;
    string strOptionalModelPath;
    string strReferenceFile;
    string strKernel;
    string strType;
    vector<string> vKernelParameters;
    float fKernelScale;
    int iNrOfBasisFunctions;
    unsigned uNumberOfDimensions;
    string strOutputFileName;
};

po::options_description initializeProgramOptions(programOptions& poParameters);
bool isOptionsConflictPresent(programOptions& opt);
template <class DataType, class RepresenterType, class DataReaderType, bool isShapeModel, unsigned Dimenstionality>
void buildAndSaveModel(programOptions opt);
void createKernelMap();
string getAvailableKernelsStr();


int main(int argc, char** argv) {
    createKernelMap();

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
            typedef itk::StandardMeshRepresenter<float, Dimensionality3D> RepresenterType;
            typedef itk::MeshFileReader<DataTypeShape> DataReaderType;
            buildAndSaveModel<DataTypeShape, RepresenterType, DataReaderType, true, Dimensionality3D>(poParameters);
        } else {
            if (poParameters.uNumberOfDimensions == 2) {
                typedef itk::StandardImageRepresenter<VectorPixel2DType, Dimensionality2D> RepresenterType;
                typedef itk::ImageFileReader<DataType2DDeformation> DataReaderType;
                buildAndSaveModel<DataType2DDeformation, RepresenterType, DataReaderType, false, Dimensionality2D>(poParameters);
            } else {
                typedef itk::StandardImageRepresenter<VectorPixel3DType, Dimensionality3D> RepresenterType;
                typedef itk::ImageFileReader<DataType3DDeformation> DataReaderType;
                buildAndSaveModel<DataType3DDeformation, RepresenterType, DataReaderType, false, Dimensionality3D>(poParameters);
            }
        }

    } catch (itk::ExceptionObject & e) {
        cerr << "Could not build the model:" << endl;
        cerr << e.what() << endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

bool isOptionsConflictPresent(programOptions& opt) {
    boost::algorithm::to_lower(opt.strKernel);
    boost::algorithm::to_lower(opt.strType);

    if (opt.strType != "shape" && opt.strType != "deformation") {
        return true;
    }

    if ((opt.strOptionalModelPath == "" && opt.strReferenceFile == "")
            || (opt.strOptionalModelPath != "" & opt.strReferenceFile != "")) {
        return true;
    }


    if (opt.strOutputFileName == "") {
        return true;
    }

    if (opt.strOutputFileName == opt.strReferenceFile) {
        return true;
    }

    if (opt.iNrOfBasisFunctions < 1) {
        return true;
    }

    if (opt.strType == "deformation" && opt.uNumberOfDimensions != 2 && opt.uNumberOfDimensions != 3) {
        return true;
    }

    if (opt.strType == "shape" && opt.uNumberOfDimensions != 3) {
        return true;
    }


    return false;
}

template <class DataType, class RepresenterType, class DataReaderType, bool isShapeModel, unsigned Dimenstionality>
void buildAndSaveModel(programOptions opt) {
    KernelMapType::const_iterator it = kernelMap.find(opt.strKernel);
    if (it == kernelMap.end()) {
        itkGenericExceptionMacro( << "The kernel '" << opt.strKernel << "' isn't available. Available kernels: " << getAvailableKernelsStr());
    }


    typedef typename DataType::PointType PointType;
    typedef boost::scoped_ptr<const statismo::ScalarValuedKernel<PointType> > MatrixPointerType;
    MatrixPointerType pKernel;
    if (isShapeModel == true) {
        pKernel.reset((statismo::ScalarValuedKernel<PointType>*) it->second.createKernelShape(opt.vKernelParameters));
    } else {
        if (Dimenstionality == Dimensionality2D) {
            pKernel.reset((statismo::ScalarValuedKernel<PointType>*) it->second.createKernel2DDeformation(opt.vKernelParameters));
        } else {
            pKernel.reset((statismo::ScalarValuedKernel<PointType>*) it->second.createKernel3DDeformation(opt.vKernelParameters));
        }
    }

    typedef boost::shared_ptr<statismo::MatrixValuedKernel<PointType> > KernelPointerType;
    KernelPointerType pUnscaledKernel(new statismo::UncorrelatedMatrixValuedKernel<PointType>(pKernel.get(), Dimenstionality));
    KernelPointerType pScaledKernel(new statismo::ScaledKernel<PointType>(pUnscaledKernel.get(), opt.fKernelScale));
    KernelPointerType pStatModelKernel;
    KernelPointerType pModelBuildingKernel;

    typedef statismo::StatisticalModel<DataType> RawModelType;
    typedef boost::shared_ptr<RawModelType> RawModelPointerType;
    typedef typename RepresenterType::DatasetPointerType DatasetPointerType;

    RawModelPointerType pRawStatisticalModel;
    typename RepresenterType::Pointer pRepresenter = RepresenterType::New();
    DatasetPointerType pMean;


    if(opt.strOptionalModelPath != "") {
        try {
            pRawStatisticalModel.reset(statismo::IO<DataType>::LoadStatisticalModel(pRepresenter.GetPointer(),
                                                                                    opt.strOptionalModelPath.c_str()));
            pStatModelKernel.reset(new statismo::StatisticalModelKernel<DataType>(pRawStatisticalModel.get()));
            pModelBuildingKernel.reset(new statismo::SumKernel<PointType>(pStatModelKernel.get(), pScaledKernel.get()));
        } catch (statismo::StatisticalModelException& s) {
            itkGenericExceptionMacro(<< "Failed to read the optional model: "<< s.what());
        }
        pMean = pRawStatisticalModel->DrawMean();
    } else {
        pModelBuildingKernel = pScaledKernel;
        typename DataReaderType::Pointer pReferenceReader = DataReaderType::New();
        pReferenceReader->SetFileName(opt.strReferenceFile);
        pReferenceReader->Update();
        pRepresenter->SetReference(pReferenceReader->GetOutput());
        pMean = pRepresenter->IdentitySample();
    }

    typedef itk::LowRankGPModelBuilder<DataType> ModelBuilderType;
    typename ModelBuilderType::Pointer gpModelBuilder = ModelBuilderType::New();
    gpModelBuilder->SetRepresenter(pRepresenter);

    typedef itk::StatisticalModel<DataType> StatisticalModelType;
    typename StatisticalModelType::Pointer pModel;
    pModel = gpModelBuilder->BuildNewModel(pMean, *pModelBuildingKernel.get(), opt.iNrOfBasisFunctions);

    itk::StatismoIO<DataType>::SaveStatisticalModel(pModel, opt.strOutputFileName.c_str());
}

string getAvailableKernelsStr() {
    string ret;
    KernelMapType::size_type mapEnd = kernelMap.size();
    KernelMapType::size_type i = 0;
    for (KernelMapType::const_iterator it = kernelMap.begin(); it != kernelMap.end(); ++it, ++i) {
        if (i + 1 == mapEnd && mapEnd > 1) {
            ret += " and ";
        } else if (i > 0) {
            ret += ", ";
        }
        ret += it->first;
    }
    return ret;
}


po::options_description initializeProgramOptions(programOptions& poParameters) {
    string kernelHelp = "Specifies the kernel (covariance function). The following kernels are available: "+getAvailableKernelsStr();

    po::options_description optMandatory("Mandatory options");
    optMandatory.add_options()
    ("type,t", po::value<string>(&poParameters.strType)->default_value("shape"), "Specifies the type of the model: shape and deformation are the two available types")
    ("dimensionality,d", po::value<unsigned>(&poParameters.uNumberOfDimensions)->default_value(3), "Dimensionality of the input image (only available if you're building a deformation model)")
    ("kernel,k", po::value<string>(&poParameters.strKernel), kernelHelp.c_str())
    ("parameters,p", po::value<vector<string> >(&poParameters.vKernelParameters)->multitoken(), "Specifies the kernel parameters. The Parameters depend on the kernel")
    ("scale,s", po::value<float>(&poParameters.fKernelScale)->default_value(1), "A Scaling factor with which the Kernel will be scaled")
    ("numberofbasisfunctions,n", po::value<int>(&poParameters.iNrOfBasisFunctions), "Number of basis functions/parameters the model will have")
    ("output-file,o", po::value<string>(&poParameters.strOutputFileName), "Name of the output file where the model will be saved")
    ;
    po::options_description optAdditional("Optional options");
    optAdditional.add_options()
    ("reference,r", po::value<string>(&poParameters.strReferenceFile), "The reference that will be used to build the model")
    ("input-model,m", po::value<string>(&poParameters.strOptionalModelPath), "Extends an existing model with data from the specified kernel. This is useful to extend existing models in case of insufficient data.")
    ("help,h", po::bool_switch(&poParameters.bDisplayHelp), "Display this help message")

    ;

    po::options_description optAllOptions;
    optAllOptions.add(optMandatory).add(optAdditional);
    return optAllOptions;
}
