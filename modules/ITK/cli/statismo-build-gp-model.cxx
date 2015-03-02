#include <iostream>

#include <itkDirectory.h>
#include <itkMeshFileReader.h>
#include <itkImageFileReader.h>

#include "itkDataManager.h"
#include "itkLowRankGPModelBuilder.h"
#include "itkStandardImageRepresenter.h"
#include "itkStandardMeshRepresenter.h"
#include "itkStatisticalModel.h"

#include <boost/program_options.hpp>

//Add new kernels in this file (and document their usage in the statismo-build-gp-model.md file)
#include "statismo-build-gp-model-kernels.h"

namespace po = boost::program_options;

struct programOptions {
    bool bDisplayHelp;
    string strReferenceFile;
    string strKernel;
    string strType;
    vector<string> vKernelParameters;
    float fKernelScale;
    int iNrOfBasisFunctions;
    string strOutputFileName;
};

po::options_description initializeProgramOptions(programOptions& poParameters);
bool isOptionsConflictPresent(programOptions& opt);
template <class DataType, class RepresenterType, class DataReaderType, bool isShapeModel>
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
            typedef itk::StandardMeshRepresenter<float, Dimensions> RepresenterType;
            typedef itk::MeshFileReader<DataTypeShape> DataReaderType;
            buildAndSaveModel<DataTypeShape, RepresenterType, DataReaderType, true>(poParameters);
        } else {
            typedef itk::StandardImageRepresenter<itk::Vector<float, Dimensions>, Dimensions> RepresenterType;
            typedef itk::ImageFileReader<DataTypeDeformation> DataReaderType;
            buildAndSaveModel<DataTypeDeformation, RepresenterType, DataReaderType, false>(poParameters);
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

    if (opt.strOutputFileName == "" || opt.strReferenceFile == "") {
        return true;
    }

    if (opt.strOutputFileName == opt.strReferenceFile) {
        return true;
    }

    if (opt.iNrOfBasisFunctions < 1) {
        return true;
    }

    return false;
}

template <class DataType, class RepresenterType, class DataReaderType, bool isShapeModel>
void buildAndSaveModel(programOptions opt) {
    typedef itk::LowRankGPModelBuilder<DataType> ModelBuilderType;
    typedef itk::StatisticalModel<DataType> StatisticalModelType;
    typedef typename DataType::PointType PointType;

    typename DataReaderType::Pointer refReader = DataReaderType::New();
    refReader->SetFileName(opt.strReferenceFile);
    refReader->Update();

    typename RepresenterType::Pointer representer = RepresenterType::New();
    representer->SetReference(refReader->GetOutput());

    KernelMapType::const_iterator it = kernelMap.find(opt.strKernel);
    if (it == kernelMap.end()) {
        itk::ExceptionObject e(__FILE__, __LINE__, "The kernel '" + opt.strKernel + "' isn't available. Available kernels: " + getAvailableKernelsStr() , ITK_LOCATION);
        throw e;
    }


    typedef boost::scoped_ptr<const statismo::ScalarValuedKernel<PointType> > MatrixPointerType;
    MatrixPointerType pKernel;
    if (isShapeModel == true) {
        pKernel.reset((statismo::ScalarValuedKernel<PointType>*) it->second.createKernelShape(opt.vKernelParameters));
    } else {
        pKernel.reset((statismo::ScalarValuedKernel<PointType>*) it->second.createKernelDeformation(opt.vKernelParameters));
    }

    const statismo::MatrixValuedKernel<PointType>& mvGk = statismo::UncorrelatedMatrixValuedKernel<PointType>(pKernel.get(), Dimensions);
    const statismo::MatrixValuedKernel<PointType>& scaledGk = statismo::ScaledKernel<PointType>(&mvGk, opt.fKernelScale);

    typename ModelBuilderType::Pointer gpModelBuilder = ModelBuilderType::New();
    gpModelBuilder->SetRepresenter(representer);

    typename StatisticalModelType::Pointer model;
    if (isShapeModel == true) {
        model = gpModelBuilder->BuildNewModel(refReader->GetOutput(), scaledGk, opt.iNrOfBasisFunctions);
    } else {
        model = gpModelBuilder->BuildNewZeroMeanModel(scaledGk, opt.iNrOfBasisFunctions);
    }

    model->Save(opt.strOutputFileName.c_str());
}

string getAvailableKernelsStr() {
    string ret;
    KernelMapType::size_type mapEnd = kernelMap.size();
    KernelMapType::size_type i = 0;
    for (KernelMapType::const_iterator it = kernelMap.begin(); it != kernelMap.end(); it++, i++) {
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
    ("kernel,k", po::value<string>(&poParameters.strKernel), kernelHelp.c_str())
    ("parameters,p", po::value<vector<string> >(&poParameters.vKernelParameters)->multitoken(), "Specifies the kernel parameters. The Parameters depend on the kernel")
    ("scale,s", po::value<float>(&poParameters.fKernelScale)->default_value(1), "A Scaling factor with which the Kernel will be scaled")
    ("numberofbasisfunctions,n", po::value<int>(&poParameters.iNrOfBasisFunctions)->default_value(25), "Number of basis functions/parameters the model will have")
    ("reference,r", po::value<string>(&poParameters.strReferenceFile), "The reference that will be used to build the model")
    ("output-file,o", po::value<string>(&poParameters.strOutputFileName), "Name of the output file where the model will be saved")
    ;
    po::options_description optAdditional("Optional options");
    optAdditional.add_options()
    ("help,h", po::bool_switch(&poParameters.bDisplayHelp), "Display this help message")
    ;

    po::options_description optAllOptions;
    optAllOptions.add(optMandatory).add(optAdditional);
    return optAllOptions;
}