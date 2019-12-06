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

#include "lpo.h"

#include "utils/itkPenalizingMeanSquaresImageToImageMetric.h"
#include "utils/statismo-fitting-utils.h"

#include <itkCommand.h>
#include <itkCompositeTransform.h>
#include <itkImageFileReader.h>
#include <itkImageRegistrationMethod.h>
#include <statismo/ITK/itkInterpolatingStatisticalDeformationModelTransform.h>
#include <itkLBFGSOptimizer.h>
#include <itkRegularStepGradientDescentOptimizer.h>
#include <itkLinearInterpolateImageFunction.h>
#include <itkNormalizedCorrelationImageToImageMetric.h>
#include <itkRigid2DTransform.h>
#include <statismo/ITK/itkStandardImageRepresenter.h>
#include <statismo/ITK/itkIO.h>
#include <statismo/ITK/itkStatisticalModel.h>
#include <itkTransformToDisplacementFieldFilter.h>
#include <itkVersorRigid3DTransform.h>
#include <itkWarpImageFilter.h>
#include <itkCenteredTransformInitializer.h>

const unsigned Dimensionality2D = 2;
const unsigned Dimensionality3D = 3;

namespace po = lpo;
using namespace std;

struct ProgramOptions
{
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
  double   dRegularizationWeight;

  bool bPrintFittingInformation;
};

bool
isOptionsConflictPresent(const ProgramOptions & opt);
template <unsigned Dimensions, class RotationAndTranslationTransformType>
void
fitImage(const ProgramOptions & opt, ConsoleOutputSilencer * pCOSilencer);

int
main(int argc, char ** argv)
{

  ProgramOptions                                      poParameters;
  lpo::program_options<std::string, double, unsigned> parser{ argv[0], "Program help:" };

  parser
    .add_opt<std::string>({ "input-model", "i", "The path to the model file.", &poParameters.strInputModelFileName },
                          true)
    .add_opt<std::string>(
      { "moving-image", "m", "The path to the moving image.", &poParameters.strInputMovingImageFileName }, true)
    .add_opt<std::string>(
      { "fixed-image", "f", "The path to the fixed image.", &poParameters.strInputFixedImageFileName }, true)
    .add_opt<unsigned>({ "dimensionality",
                         "d",
                         "Dimensionality of the input image and model",
                         &poParameters.uNumberOfDimensions,
                         3,
                         2,
                         3 },
                       true)
    .add_opt<unsigned>({ "number-of-iterations", "n", "Number of iterations", &poParameters.uNumberOfIterations, 100 },
                       true)
    .add_opt<double>(
      { "regularization-weight",
        "w",
        "This is the regularization weight to make sure the model parameters don't don't get too big while fitting.",
        &poParameters.dRegularizationWeight },
      true)
    .

    add_opt<std::string>({ "output-model-deformationfield",
                           "a",
                           "Name of the output file where the model deformation field will be written to.",
                           &poParameters.strOutputModelTransformFileName })
    .add_opt<std::string>({ "output-deformationfield",
                            "e",
                            "Name of the output file where the entire deformation field will be written to. This "
                            "includes the rotation and translation (Only use this when NOT using landmarks).",
                            &poParameters.strOutputEntireTransformFileName })
    .

    add_opt<std::string>({ "fixed-landmarks",
                           "",
                           "Name of the file where the fixed Landmarks are saved.",
                           &poParameters.strInputFixedLandmarksFileName })
    .add_opt<std::string>({ "moving-landmarks",
                            "",
                            "Name of the file where the moving Landmarks are saved.",
                            &poParameters.strInputMovingLandmarksFileName })
    .add_opt<double>({ "landmarks-variance",
                       "v",
                       "The variance that will be used to build the posterior model.",
                       &poParameters.dLandmarksVariance,
                       1.0f })
    .add_flag(
      { "print-fitting-information",
        "p",
        "Prints information (the parameters, metric score and the iteration count) with each iteration while fitting.",
        &poParameters.bPrintFittingInformation })
    .add_pos_opt<std::string>({ "Name of the output file where the fitted image will be written to.",
                                &poParameters.strOutputFittedImageFileName });

  if (!parser.parse(argc, argv))
  {
    return EXIT_FAILURE;
  }

  if (isOptionsConflictPresent(poParameters))
  {
    cerr << "A conflict in the options exists or insufficient options were set." << endl;
    cout << parser << endl;
    return EXIT_FAILURE;
  }

  ConsoleOutputSilencer coSilencer;
  try
  {
    if (poParameters.uNumberOfDimensions == Dimensionality2D)
    {
      typedef itk::Rigid2DTransform<double> RotationAndTranslationTransformType;
      fitImage<Dimensionality2D, RotationAndTranslationTransformType>(poParameters, &coSilencer);
    }
    else
    {
      typedef itk::VersorRigid3DTransform<double> RotationAndTranslationTransformType;
      fitImage<Dimensionality3D, RotationAndTranslationTransformType>(poParameters, &coSilencer);
    }
  }
  catch (ifstream::failure & e)
  {
    coSilencer.enableOutput();
    cerr << "Could not read a file:" << endl;
    cerr << e.what() << endl;
    return EXIT_FAILURE;
  }
  catch (itk::ExceptionObject & e)
  {
    coSilencer.enableOutput();
    cerr << "Could not fit the model:" << endl;
    cerr << e.what() << endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

bool
isOptionsConflictPresent(const ProgramOptions & opt)
{
  // if one set of the landmarks-file is provided, then both have to be provided (-> use XOR)
  if (((opt.strInputFixedLandmarksFileName != "") ^ (opt.strInputMovingLandmarksFileName != "")) == true)
  {
    return true;
  }

  if (opt.strInputFixedImageFileName == "" || opt.strInputModelFileName == "" || opt.strInputMovingImageFileName == "")
  {
    return true;
  }

  // at least one thing has to be saved/displayed, otherwise running it is pointless
  if (opt.strOutputEntireTransformFileName == "" && opt.strOutputFittedImageFileName == "" &&
      opt.strOutputModelTransformFileName == "")
  {
    return true;
  }


  return false;
}

template <class ImageType>
void
saveImage(typename ImageType::Pointer pImage, const std::string & strOutputFileName)
{
  typedef itk::ImageFileWriter<ImageType> ImageWriter;
  typename ImageWriter::Pointer           pImageWriter = ImageWriter::New();
  pImageWriter->SetInput(pImage);
  pImageWriter->SetFileName(strOutputFileName.c_str());
  pImageWriter->Update();
}

template <class DisplacementFieldImageType, class ReferenceImageType, class TransformType>
typename DisplacementFieldImageType::Pointer
generateAndSaveDisplacementField(typename ReferenceImageType::Pointer pReferenceImage,
                                 typename TransformType::Pointer      pTransform,
                                 const std::string &                  strOutputFileName)
{
  typedef itk::TransformToDisplacementFieldFilter<DisplacementFieldImageType, double> DisplacementFieldGeneratorType;
  typename DisplacementFieldGeneratorType::Pointer pDispfieldGenerator = DisplacementFieldGeneratorType::New();
  pDispfieldGenerator->UseReferenceImageOn();
  pDispfieldGenerator->SetReferenceImage(pReferenceImage);

  pDispfieldGenerator->SetTransform(pTransform);
  pDispfieldGenerator->Update();

  typename DisplacementFieldImageType::Pointer pDisplacementField = pDispfieldGenerator->GetOutput();

  if (strOutputFileName != "")
  {
    saveImage<DisplacementFieldImageType>(pDisplacementField, strOutputFileName);
  }

  return pDisplacementField;
}

template <unsigned Dimensions, class RotationAndTranslationTransformType>
void
fitImage(const ProgramOptions & opt, ConsoleOutputSilencer * pCOSilencer)
{
  typedef itk::Image<float, Dimensions>   ImageType;
  typedef itk::ImageFileReader<ImageType> ImageReaderType;
  typename ImageReaderType::Pointer       pFixedImageReader = ImageReaderType::New();
  pFixedImageReader->SetFileName(opt.strInputFixedImageFileName.c_str());
  pFixedImageReader->Update();
  typename ImageType::Pointer pFixedImage = pFixedImageReader->GetOutput();

  typename ImageReaderType::Pointer pMovingImageReader = ImageReaderType::New();
  pMovingImageReader->SetFileName(opt.strInputMovingImageFileName.c_str());
  pMovingImageReader->Update();
  typename ImageType::Pointer pMovingImage = pMovingImageReader->GetOutput();

  typedef itk::Vector<float, Dimensions>                             VectorPixelType;
  typedef itk::Image<VectorPixelType, Dimensions>                    VectorImageType;
  typedef itk::StandardImageRepresenter<VectorPixelType, Dimensions> RepresenterType;
  typename RepresenterType::Pointer                                  pRepresenter = RepresenterType::New();

  typedef itk::StatisticalModel<VectorImageType> StatisticalModelType;
  typename StatisticalModelType::Pointer         pModel = StatisticalModelType::New();
  pModel = itk::StatismoIO<VectorImageType>::LoadStatisticalModel(pRepresenter, opt.strInputModelFileName.c_str());

  typedef itk::Transform<double, Dimensions, Dimensions> TransformType;
  typename TransformType::Pointer                        pTransform;

  typedef itk::InterpolatingStatisticalDeformationModelTransform<VectorImageType, double, Dimensions>
                                       ModelTransformType;
  typename ModelTransformType::Pointer pModelTransform = ModelTransformType::New();

  if (opt.strInputMovingLandmarksFileName != "")
  {
    pModel = buildPosteriorDeformationModel<VectorImageType, StatisticalModelType>(
      pModel, opt.strInputFixedLandmarksFileName, opt.strInputMovingLandmarksFileName, opt.dLandmarksVariance);
    pTransform = pModelTransform;
  }
  else
  {
    // No Landmarks are available: we also have to allow rotation and translation.
    typename RotationAndTranslationTransformType::Pointer pRotationAndTranslationTransform =
      RotationAndTranslationTransformType::New();
    pRotationAndTranslationTransform->SetIdentity();


    using TransformInitializerType =
      itk::CenteredTransformInitializer<RotationAndTranslationTransformType, ImageType, ImageType>;
    auto initializer = TransformInitializerType::New();
    initializer->SetTransform(pRotationAndTranslationTransform);
    initializer->SetFixedImage(pFixedImage);
    initializer->SetMovingImage(pMovingImage);
    initializer->MomentsOn();
    initializer->InitializeTransform();

    typedef itk::CompositeTransform<double, Dimensions> CompositeTransformType;
    typename CompositeTransformType::Pointer            pCompositeTransform = CompositeTransformType::New();
    pCompositeTransform->AddTransform(pRotationAndTranslationTransform);
    pCompositeTransform->AddTransform(pModelTransform);
    pCompositeTransform->SetAllTransformsToOptimizeOn();

    pTransform = pCompositeTransform;
  }

  pModelTransform->SetStatisticalModel(pModel);
  pModelTransform->SetIdentity();

  typedef itk::LBFGSOptimizer OptimizerType;
  OptimizerType::Pointer      pOptimizer = OptimizerType::New();
  initializeOptimizer<OptimizerType>(pOptimizer,
                                     opt.uNumberOfIterations,
                                     pModel->GetNumberOfPrincipalComponents(),
                                     pTransform->GetNumberOfParameters(),
                                     opt.bPrintFittingInformation,
                                     pCOSilencer);

  typedef itk::PenalizingMeanSquaresImageToImageMetric<ImageType, ImageType> MetricType;
  typename MetricType::Pointer                                               pMetric = MetricType::New();
  pMetric->SetRegularizationParameter(opt.dRegularizationWeight);
  pMetric->SetNumberOfModelComponents(pModel->GetNumberOfPrincipalComponents());

  typedef itk::LinearInterpolateImageFunction<ImageType, double> InterpolatorType;
  typename InterpolatorType::Pointer                             pInterpolator = InterpolatorType::New();

  typedef itk::ImageRegistrationMethod<ImageType, ImageType> RegistrationFilterType;
  typename RegistrationFilterType::Pointer                   pRegistration = RegistrationFilterType::New();
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

  typename VectorImageType::Pointer pDisplacementField =
    generateAndSaveDisplacementField<VectorImageType, ImageType, TransformType>(
      pFixedImage, pTransform, opt.strOutputEntireTransformFileName);
  generateAndSaveDisplacementField<VectorImageType, ImageType, TransformType>(
    pFixedImage, (typename TransformType::Pointer)pModelTransform, opt.strOutputModelTransformFileName);

  if (opt.strOutputFittedImageFileName != "")
  {
    typedef itk::WarpImageFilter<ImageType, ImageType, VectorImageType> WarpFilterType;
    typename WarpFilterType::Pointer                                    pWarper = WarpFilterType::New();
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
