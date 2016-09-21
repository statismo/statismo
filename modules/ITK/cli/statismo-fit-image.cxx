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

#include <itkCommand.h>
#include <itkCompositeTransform.h>
#include <itkImageFileReader.h>
#include <itkImageRegistrationMethod.h>
#include <itkInterpolatingStatisticalDeformationModelTransform.h>
#include <itkLBFGSOptimizer.h>
#include <itkLinearInterpolateImageFunction.h>
#include <itkNormalizedCorrelationImageToImageMetric.h>
#include <itkRigid2DTransform.h>
#include <itkStandardImageRepresenter.h>
#include <itkStatismoIO.h>
#include <itkStatisticalModel.h>
#if (ITK_VERSION_MAJOR >= 4 && ITK_VERSION_MINOR >= 6)
#include <itkTransformToDisplacementFieldFilter.h>
#else
#include <itkTransformToDisplacementFieldSource.h>
#endif
#include <itkVersorRigid3DTransform.h>
#include <itkWarpImageFilter.h>

#include "utils/itkPenalizingMeanSquaresImageToImageMetric.h"
#include "utils/statismo-fitting-utils.h"

const unsigned Dimensionality2D = 2;
const unsigned Dimensionality3D = 3;

namespace po = boost::program_options;
using namespace std;

struct programOptions {
    bool bDisplayHelp;

    string strInputModelFileName;
    string strInputMovingImageFileName;
    string strInputFixedImageFileName;

    string strOutputFittedImageFileName;
    string strOutputModelTransformFileName;
    string strOutputEntireTransformFileName;

    string strInputFixedLandmarksFileName;
    string strInputMovingLandmarksFileName;
    double dLandmarksVariance;

    unsigned uNumberOfDimensions;
    unsigned uNumberOfIterations;
    double dRegularizationWeight;

    bool bPrintFittingInformation;
};

po::options_description initializeProgramOptions(programOptions& poParameters);
bool isOptionsConflictPresent(programOptions& opt);
template<unsigned Dimensions, class RotationAndTranslationTransformType>
void fitImage(programOptions opt, ConsoleOutputSilencer* pCOSilencer);



int main(int argc, char** argv) {
    po::positional_options_description optPositional;
    optPositional.add("output-fit", 1);

    programOptions poParameters;
    po::options_description optAllOptions = initializeProgramOptions(poParameters);


    po::variables_map vm;
    try {
        po::parsed_options parsedOptions = po::command_line_parser(argc, argv).options(optAllOptions).positional(optPositional).run();
        po::store(parsedOptions, vm);
        po::notify(vm);
    } catch (po::error& e) {
        cerr << "An exception occurred while parsing the Command line:"<<endl;
        cerr << e.what() << endl << endl;
        cout << optAllOptions << endl;
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

    ConsoleOutputSilencer coSilencer;
    try {
        if (poParameters.uNumberOfDimensions == Dimensionality2D) {
            typedef itk::Rigid2DTransform<double> RotationAndTranslationTransformType;
            fitImage<Dimensionality2D, RotationAndTranslationTransformType>(poParameters, &coSilencer);
        } else {
            typedef itk::VersorRigid3DTransform<double> RotationAndTranslationTransformType;
            fitImage<Dimensionality3D, RotationAndTranslationTransformType>(poParameters, &coSilencer);
        }
    } catch (ifstream::failure & e) {
        coSilencer.enableOutput();
        cerr << "Could not read a file:" << endl;
        cerr << e.what() << endl;
        return EXIT_FAILURE;
    } catch (itk::ExceptionObject & e) {
        coSilencer.enableOutput();
        cerr << "Could not fit the model:" << endl;
        cerr << e.what() << endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

bool isOptionsConflictPresent(programOptions& opt) {
    //if one set of the landmarks-file is provided, then both have to be provided (-> use XOR)
    if (((opt.strInputFixedLandmarksFileName != "") ^ (opt.strInputMovingLandmarksFileName != "")) == true) {
        return true;
    }

    if (opt.strInputFixedImageFileName == "" || opt.strInputModelFileName == "" || opt.strInputMovingImageFileName == "") {
        return true;
    }

    if (opt.uNumberOfDimensions != 2 && opt.uNumberOfDimensions != 3) {
        return true;
    }

    //at least one thing has to be saved/displayed, otherwise running it is pointless
    if (opt.strOutputEntireTransformFileName == "" && opt.strOutputFittedImageFileName == "" && opt.strOutputModelTransformFileName == "") {
        return true;
    }


    return false;
}

template<class ImageType>
void saveImage(typename ImageType::Pointer pImage, const std::string& strOutputFileName) {
    typedef itk::ImageFileWriter<ImageType>  ImageWriter;
    typename ImageWriter::Pointer pImageWriter = ImageWriter::New();
    pImageWriter->SetInput(pImage);
    pImageWriter->SetFileName(strOutputFileName.c_str());
    pImageWriter->Update();
}

template<class DisplacementFieldImageType, class ReferenceImageType, class TransformType>
typename DisplacementFieldImageType::Pointer generateAndSaveDisplacementField(typename ReferenceImageType::Pointer pReferenceImage, typename TransformType::Pointer pTransform, const std::string& strOutputFileName) {
#if (ITK_VERSION_MAJOR >= 4 && ITK_VERSION_MINOR >= 6)
    typedef itk::TransformToDisplacementFieldFilter<DisplacementFieldImageType,	double>  DisplacementFieldGeneratorType;
    typename DisplacementFieldGeneratorType::Pointer pDispfieldGenerator = DisplacementFieldGeneratorType::New();
    pDispfieldGenerator->UseReferenceImageOn();
    pDispfieldGenerator->SetReferenceImage(pReferenceImage);
#else
    typedef typename itk::TransformToDisplacementFieldSource<DisplacementFieldImageType, double> DisplacementFieldGeneratorType;
    typename DisplacementFieldGeneratorType::Pointer pDispfieldGenerator = DisplacementFieldGeneratorType::New();
    pDispfieldGenerator->SetOutputParametersFromImage(pReferenceImage);
#endif

    pDispfieldGenerator->SetTransform(pTransform);
    pDispfieldGenerator->Update();

    typename DisplacementFieldImageType::Pointer pDisplacementField = pDispfieldGenerator->GetOutput();

    if (strOutputFileName != "") {
        saveImage<DisplacementFieldImageType>(pDisplacementField, strOutputFileName);
    }

    return pDisplacementField;
}


template<unsigned Dimensions, class RotationAndTranslationTransformType>
void fitImage(programOptions opt, ConsoleOutputSilencer* pCOSilencer) {
    typedef itk::Image<float, Dimensions> ImageType;
    typedef itk::ImageFileReader<ImageType> ImageReaderType;
    typename ImageReaderType::Pointer pFixedImageReader = ImageReaderType::New();
    pFixedImageReader->SetFileName(opt.strInputFixedImageFileName.c_str());
    pFixedImageReader->Update();
    typename ImageType::Pointer pFixedImage = pFixedImageReader->GetOutput();

    typename ImageReaderType::Pointer pMovingImageReader = ImageReaderType::New();
    pMovingImageReader->SetFileName(opt.strInputMovingImageFileName.c_str());
    pMovingImageReader->Update();
    typename ImageType::Pointer pMovingImage = pMovingImageReader->GetOutput();

    typedef itk::Vector<float, Dimensions> VectorPixelType;
    typedef itk::Image<VectorPixelType, Dimensions> VectorImageType;
    typedef itk::StandardImageRepresenter<VectorPixelType, Dimensions> RepresenterType;
    typename RepresenterType::Pointer pRepresenter = RepresenterType::New();

    typedef itk::StatisticalModel<VectorImageType> StatisticalModelType;
    typename StatisticalModelType::Pointer pModel = StatisticalModelType::New();
    pModel = itk::StatismoIO<VectorImageType>::LoadStatisticalModel(pRepresenter, opt.strInputModelFileName.c_str());

    typedef itk::Transform<double, Dimensions, Dimensions> TransformType;
    typename TransformType::Pointer pTransform;

    typedef itk::InterpolatingStatisticalDeformationModelTransform<VectorImageType, double, Dimensions> ModelTransformType;
    typename ModelTransformType::Pointer pModelTransform = ModelTransformType::New();

    if (opt.strInputMovingLandmarksFileName != "") {
        pModel = buildPosteriorDeformationModel<VectorImageType, StatisticalModelType>(pModel, opt.strInputFixedLandmarksFileName, opt.strInputMovingLandmarksFileName, opt.dLandmarksVariance);
        pTransform = pModelTransform;
    } else {
        //No Landmarks are available: we also have to allow rotation and translation.
        typename RotationAndTranslationTransformType::Pointer pRotationAndTranslationTransform = RotationAndTranslationTransformType::New();
        pRotationAndTranslationTransform->SetIdentity();

        typedef itk::CompositeTransform<double, Dimensions> CompositeTransformType;
        typename CompositeTransformType::Pointer pCompositeTransform = CompositeTransformType::New();
        pCompositeTransform->AddTransform(pRotationAndTranslationTransform);
        pCompositeTransform->AddTransform(pModelTransform);
        pCompositeTransform->SetAllTransformsToOptimizeOn();

        pTransform = pCompositeTransform;
    }

    pModelTransform->SetStatisticalModel(pModel);
    pModelTransform->SetIdentity();

    typedef itk::LBFGSOptimizer OptimizerType;
    OptimizerType::Pointer pOptimizer = OptimizerType::New();
    initializeOptimizer<OptimizerType>(pOptimizer, opt.uNumberOfIterations, pModel->GetNumberOfPrincipalComponents(), pTransform->GetNumberOfParameters(), opt.bPrintFittingInformation, pCOSilencer);

    typedef itk::PenalizingMeanSquaresImageToImageMetric<ImageType, ImageType> MetricType;
    typename MetricType::Pointer pMetric = MetricType::New();
    pMetric->SetRegularizationParameter(opt.dRegularizationWeight);
    pMetric->SetNumberOfModelComponents(pModel->GetNumberOfPrincipalComponents());

    typedef itk::LinearInterpolateImageFunction<ImageType, double> InterpolatorType;
    typename InterpolatorType::Pointer pInterpolator = InterpolatorType::New();

    typedef itk::ImageRegistrationMethod<ImageType, ImageType> RegistrationFilterType;
    typename RegistrationFilterType::Pointer pRegistration = RegistrationFilterType::New();
    pRegistration->SetInitialTransformParameters(pTransform->GetParameters());
    pRegistration->SetMetric(pMetric);
    pRegistration->SetOptimizer(pOptimizer);
    pRegistration->SetTransform(pTransform);
    pRegistration->SetInterpolator(pInterpolator);
    pRegistration->SetFixedImage(pFixedImage);
    pRegistration->SetFixedImageRegion(pFixedImage->GetBufferedRegion());
    pRegistration->SetMovingImage(pMovingImage);

    pCOSilencer->disableOutput();
    pRegistration->Update();
    pCOSilencer->enableOutput();

    typename VectorImageType::Pointer pDisplacementField = generateAndSaveDisplacementField<VectorImageType, ImageType, TransformType>(
                pFixedImage, pTransform, opt.strOutputEntireTransformFileName);
    generateAndSaveDisplacementField<VectorImageType, ImageType, TransformType>(pFixedImage, (typename TransformType::Pointer) pModelTransform, opt.strOutputModelTransformFileName);

    if (opt.strOutputFittedImageFileName != "") {
        typedef itk::WarpImageFilter<ImageType, ImageType, VectorImageType> WarpFilterType;
        typename WarpFilterType::Pointer pWarper = WarpFilterType::New();
        pWarper->SetInput(pMovingImage);
        pWarper->SetInterpolator(pInterpolator);
        pWarper->SetOutputSpacing(pFixedImage->GetSpacing());
        pWarper->SetOutputOrigin(pFixedImage->GetOrigin());
        pWarper->SetOutputDirection(pFixedImage->GetDirection());
        pWarper->SetDisplacementField(pDisplacementField);
        pWarper->Update();

        saveImage<ImageType>(pWarper->GetOutput(), opt.strOutputFittedImageFileName);
    }
}


po::options_description initializeProgramOptions(programOptions& poParameters) {
    po::options_description optMandatory("Mandatory options");
    optMandatory.add_options()

    ("input-model,i", po::value<string>(&poParameters.strInputModelFileName), "The path to the model file.")
    ("moving-image,m", po::value<string>(&poParameters.strInputMovingImageFileName), "The path to the moving image.")
    ("fixed-image,f", po::value<string>(&poParameters.strInputFixedImageFileName), "The path to the fixed image.")
    ("dimensionality,d", po::value<unsigned>(&poParameters.uNumberOfDimensions)->default_value(3), "Dimensionality of the input images & model")
    ("number-of-iterations,n", po::value<unsigned>(&poParameters.uNumberOfIterations)->default_value(100), "Number of iterations")
    ("regularization-weight,w", po::value<double>(&poParameters.dRegularizationWeight), "This is the regularization weight to make sure the model parameters don't don't get too big while fitting.")
    ;

    po::options_description optOutput("Mandatory Output options (set at least one)");
    optOutput.add_options()
    ("output-fit,o", po::value<string>(&poParameters.strOutputFittedImageFileName), "Name of the output file where the fitted image will be written to.")
    ("output-model-deformationfield,a", po::value<string>(&poParameters.strOutputModelTransformFileName), "Name of the output file where the model deformation field will be written to.")
    ("output-deformationfield,e", po::value<string>(&poParameters.strOutputEntireTransformFileName), "Name of the output file where the entire deformation field will be written to. This includes the rotation and translation (Only use this when NOT using landmarks).")
    ;

    po::options_description optLandmarks("Landmarks (optional - if you set one you have to set all)");
    optLandmarks.add_options()
    ("fixed-landmarks", po::value<string>(&poParameters.strInputFixedLandmarksFileName), "Name of the file where the fixed Landmarks are saved.")
    ("moving-landmarks", po::value<string>(&poParameters.strInputMovingLandmarksFileName), "Name of the file where the moving Landmarks are saved.")
    ("landmarks-variance,v", po::value<double>(&poParameters.dLandmarksVariance)->default_value(1), "The variance that will be used to build the posterior model.")
    ;

    po::options_description optAdditional("Optional options");
    optAdditional.add_options()
    ("print-fitting-information,p", po::bool_switch(&poParameters.bPrintFittingInformation), "Prints information (the parameters, metric score and the iteration count) with each iteration while fitting.")
    ("help,h", po::bool_switch(&poParameters.bDisplayHelp), "Display this help message")
    ;

    po::options_description optAllOptions;
    optAllOptions.add(optMandatory).add(optOutput).add(optLandmarks).add(optAdditional);
    return optAllOptions;
}
