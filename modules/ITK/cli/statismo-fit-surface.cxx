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

#include "utils/itkPenalizingMeanSquaresPointSetToImageMetric.h"
#include "utils/statismo-build-models-utils.h"
#include "utils/statismo-fitting-utils.h"

#include <itkBoundingBox.h>
#include <itkCompositeTransform.h>
#include <itkSignedMaurerDistanceMapImageFilter.h>
#include <itkLBFGSOptimizer.h>
#include <itkLinearInterpolateImageFunction.h>
#include <itkMesh.h>
#include <itkPointSetToImageFilter.h>
#include <itkPointSetToImageRegistrationMethod.h>
#include <itkPointsLocator.h>
#include <statismo/ITK/itkReducedVarianceModelBuilder.h>
#include <statismo/ITK/itkStandardMeshRepresenter.h>
#include <statismo/ITK/itkIO.h>
#include <statismo/ITK/itkStatisticalModel.h>
#include <statismo/ITK/itkStatisticalShapeModelTransform.h>
#include <itkVersorRigid3DTransform.h>

namespace po = lpo;
using namespace std;

const unsigned                          Dimensions = 3;
typedef itk::Mesh<float, Dimensions>    DataType;
typedef itk::Image<float, Dimensions>   DistanceImageType;
typedef itk::StatisticalModel<DataType> StatisticalModelType;

struct ProgramOptions
{
  bool bPrintFittingInformation;

  string strInputModelFileName;
  string strInputTargetMeshFileName;

  string strOutputFittedMeshFileName;
  string strOutputProjectedMeshFileName;

  double   dRegularizationWeight;
  unsigned uNumberOfIterations;
  string   strInputFixedLandmarksFileName;
  string   strInputMovingLandmarksFileName;
  double   dLandmarksVariance;
};

bool
isOptionsConflictPresent(const ProgramOptions & opt);
void
fitMesh(const ProgramOptions & opt, ConsoleOutputSilencer * pCOSilencer);

int
main(int argc, char ** argv)
{

  ProgramOptions                                      poParameters;
  lpo::program_options<std::string, double, unsigned> parser{ argv[0], "Program help:" };

  parser
    .add_opt<std::string>({ "input-model", "i", "The path to the model file.", &poParameters.strInputModelFileName },
                          true)
    .add_opt<std::string>(
      { "input-targetmesh", "t", "The path to the target mesh.", &poParameters.strInputTargetMeshFileName }, true)
    .add_opt<unsigned>({ "number-of-iterations", "n", "Number of iterations", &poParameters.uNumberOfIterations, 100 },
                       true)
    .add_opt<double>(
      { "regularization-weight",
        "w",
        "This is the regularization weight to make sure the model parameters don't don't get too big while fitting.",
        &poParameters.dRegularizationWeight },
      true)
    .

    add_opt<std::string>({ "output-fit",
                           "o",
                           "Name of the output file where the fitted mesh will be written to..",
                           &poParameters.strOutputFittedMeshFileName })
    .add_opt<std::string>({ "output-projected",
                            "j",
                            "Name of the output file where the projected mesh will be written to.",
                            &poParameters.strOutputProjectedMeshFileName })
    .

    add_opt<std::string>({ "fixed-landmarks",
                           "f",
                           "Name of the file where the fixed Landmarks are saved.",
                           &poParameters.strInputFixedLandmarksFileName })
    .add_opt<std::string>({ "moving-landmarks",
                            "m",
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
        &poParameters.bPrintFittingInformation });

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
    fitMesh(poParameters, &coSilencer);
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

  // at least one of the outputs needs to be set or the fitting information has to be printed
  if (opt.strOutputFittedMeshFileName == "" && opt.strOutputProjectedMeshFileName == "")
  {
    return true;
  }

  if (opt.strInputModelFileName == "" || opt.strInputTargetMeshFileName == "")
  {
    return true;
  }

  return false;
}

DistanceImageType::Pointer
computeDistanceImageForMesh(DataType::Pointer pMesh, const unsigned uDistImageResolution = 256)
{
  // Compute a bounding box around the reference shape
  typedef itk::BoundingBox<int, Dimensions, float, DataType::PointsContainer> BoundingBoxType;
  BoundingBoxType::Pointer                                                    pBoundingBox = BoundingBoxType::New();
  pBoundingBox->SetPoints(pMesh->GetPoints());
  pBoundingBox->ComputeBoundingBox();

  // Compute a binary image from the point set, which is as large as the bounding box plus a margin.
  typedef itk::Image<unsigned char, Dimensions>                     BinaryImageType;
  typedef itk::PointSet<float, Dimensions>                          PointSetType;
  typedef itk::PointSetToImageFilter<PointSetType, BinaryImageType> PointsToImageFilterType;
  PointsToImageFilterType::Pointer pointsToImageFilter = PointsToImageFilterType::New();
  pointsToImageFilter->SetInput(pMesh);
  BinaryImageType::SpacingType spacing;
  BinaryImageType::SpacingType margin;
  BinaryImageType::PointType   origin = pBoundingBox->GetMinimum();
  BinaryImageType::SpacingType diff = pBoundingBox->GetMaximum() - pBoundingBox->GetMinimum();
  BinaryImageType::SizeType    size;

  for (unsigned i = 0; i < Dimensions; ++i)
  {
    margin[i] = diff[i] * 0.1; // 10 % margin on each side
    origin[i] -= margin[i];
    size[i] = uDistImageResolution + margin[i];
    spacing[i] = (diff[i] + 2.0 * margin[i]) / uDistImageResolution;
  }

  pointsToImageFilter->SetSpacing(spacing);
  pointsToImageFilter->SetOrigin(origin);
  pointsToImageFilter->SetSize(size);
  pointsToImageFilter->Update();

  // compute a distance map to the points in the pointset
  BinaryImageType::Pointer pBinaryImage = pointsToImageFilter->GetOutput();
  typedef itk::SignedMaurerDistanceMapImageFilter<BinaryImageType, DistanceImageType> DistanceFilterType;
  DistanceFilterType::Pointer distanceFilter = DistanceFilterType::New();
  distanceFilter->SetInput(pBinaryImage);
  distanceFilter->Update();
  DistanceImageType::Pointer pDistanceImage = distanceFilter->GetOutput();
  return pDistanceImage;
}

void
saveMesh(DataType::Pointer pData, const string & strFileName)
{
  typedef itk::MeshFileWriter<DataType> DataWriterType;
  DataWriterType::Pointer               pDataWriter = DataWriterType::New();
  pDataWriter->SetFileName(strFileName);
  pDataWriter->SetInput(pData);
  pDataWriter->Update();
}

template <class PointsLocatorType>
DataType::Pointer
projectOnTargetMesh(DataType::Pointer pMesh, DataType::Pointer pTargetMesh)
{
  typename PointsLocatorType::Pointer ptLocator = PointsLocatorType::New();
  ptLocator->SetPoints(pTargetMesh->GetPoints());
  ptLocator->Initialize();
  DataType::Pointer projectedMesh = cloneMesh<DataType>(pMesh);
  for (DataType::PointsContainer::Iterator pointIter = projectedMesh->GetPoints()->Begin();
       pointIter != projectedMesh->GetPoints()->End();
       ++pointIter)
  {
    unsigned uClosestPointId = ptLocator->FindClosestPoint(pointIter->Value());
    pointIter->Value() = pTargetMesh->GetPoint(uClosestPointId);
  }
  return projectedMesh;
}

void
fitMesh(const ProgramOptions & opt, ConsoleOutputSilencer * pCOSilencer)
{
  typedef itk::MeshFileReader<DataType> MeshReaderType;
  MeshReaderType::Pointer               pMeshReader = MeshReaderType::New();
  pMeshReader->SetFileName(opt.strInputTargetMeshFileName.c_str());
  pMeshReader->Update();
  DataType::Pointer pTargetMesh = pMeshReader->GetOutput();

  typedef itk::StandardMeshRepresenter<float, Dimensions> RepresenterType;
  RepresenterType::Pointer                                pRepresenter = RepresenterType::New();
  StatisticalModelType::Pointer                           pModel = StatisticalModelType::New();
  pModel =
    itk::StatismoIO<DataType>::LoadStatisticalModel(pRepresenter.GetPointer(), opt.strInputModelFileName.c_str());

  StatisticalModelType::Pointer                                             pConstrainedModel;
  typedef itk::StatisticalShapeModelTransform<DataType, double, Dimensions> StatisticalModelTransformType;
  typedef itk::Transform<double, Dimensions, Dimensions>                    TransformType;
  TransformType::Pointer                                                    pTransform;

  typedef itk::PointsLocator<DataType::PointsContainer> PointsLocatorType;
  if (opt.strInputFixedLandmarksFileName == "")
  {
    typedef itk::BoundingBox<int, Dimensions, float, DataType::PointsContainer> BoundingBoxType;
    // Compute bounding box of model mesh
    BoundingBoxType::Pointer pModelMeshBox = BoundingBoxType::New();
    pModelMeshBox->SetPoints(pModel->GetRepresenter()->GetReference()->GetPoints());
    pModelMeshBox->ComputeBoundingBox();

    // Compute bounding box of target mesh
    BoundingBoxType::Pointer pTargetMeshBox = BoundingBoxType::New();
    pTargetMeshBox->SetPoints(pTargetMesh->GetPoints());
    pTargetMeshBox->ComputeBoundingBox();

    pConstrainedModel = pModel;
    StatisticalModelTransformType::Pointer pModelTransform = StatisticalModelTransformType::New();
    pModelTransform->SetStatisticalModel(pModel);
    pModelTransform->SetIdentity();

    // No Landmarks are available: we also have to allow rotation and translation.
    typedef itk::VersorRigid3DTransform<double>  RotationAndTranslationTransformType;
    RotationAndTranslationTransformType::Pointer pRotationAndTranslationTransform =
      RotationAndTranslationTransformType::New();
    pRotationAndTranslationTransform->SetIdentity();

    RotationAndTranslationTransformType::CenterType center = pModelMeshBox->GetCenter();
    pRotationAndTranslationTransform->SetCenter(center);

    RotationAndTranslationTransformType::TranslationType translation =
      pTargetMeshBox->GetCenter() - pModelMeshBox->GetCenter();
    pRotationAndTranslationTransform->SetTranslation(translation);

    typedef itk::CompositeTransform<double, Dimensions> CompositeTransformType;
    CompositeTransformType::Pointer                     pCompositeTransform = CompositeTransformType::New();
    pCompositeTransform->AddTransform(pRotationAndTranslationTransform);
    pCompositeTransform->AddTransform(pModelTransform);
    pCompositeTransform->SetAllTransformsToOptimizeOn();

    pTransform = pCompositeTransform;
  }
  else
  {
    pConstrainedModel = buildPosteriorShapeModel<DataType, StatisticalModelType, PointsLocatorType>(
      pModel, opt.strInputFixedLandmarksFileName, opt.strInputMovingLandmarksFileName, opt.dLandmarksVariance);
    StatisticalModelTransformType::Pointer pModelTransform = StatisticalModelTransformType::New();
    pModelTransform->SetStatisticalModel(pConstrainedModel);
    pModelTransform->SetIdentity();
    pTransform = pModelTransform;
  }


  DistanceImageType::Pointer pDistanceImage = computeDistanceImageForMesh(pTargetMesh);

  typedef itk::LBFGSOptimizer OptimizerType;
  OptimizerType::Pointer      pOptimizer = OptimizerType::New();
  initializeOptimizer<OptimizerType>(pOptimizer,
                                     opt.uNumberOfIterations,
                                     pModel->GetNumberOfPrincipalComponents(),
                                     pTransform->GetNumberOfParameters(),
                                     opt.bPrintFittingInformation,
                                     pCOSilencer);

  typedef itk::LinearInterpolateImageFunction<DistanceImageType, double> InterpolatorType;
  InterpolatorType::Pointer                                              pInterpolator = InterpolatorType::New();

  typedef itk::PointSet<float, Dimensions>                                                 PointSetType;
  typedef itk::PenalizingMeanSquaresPointSetToImageMetric<PointSetType, DistanceImageType> MetricType;
  MetricType::Pointer                                                                      pMetric = MetricType::New();
  pMetric->SetRegularizationParameter(opt.dRegularizationWeight);
  pMetric->SetNumberOfModelComponents(pConstrainedModel->GetNumberOfPrincipalComponents());

  DataType::Pointer     pReference = pModel->GetRepresenter()->GetReference();
  PointSetType::Pointer pFixedPointSet = PointSetType::New();
  pFixedPointSet->SetPoints(pReference->GetPoints());
  PointSetType::PointDataContainer::Pointer pPoints = PointSetType::PointDataContainer::New();
  pPoints->Reserve(pReference->GetNumberOfPoints());
  for (PointSetType::PointDataContainer::Iterator it = pPoints->Begin(); it != pPoints->End(); ++it)
  {
    it->Value() = 0;
  }
  pFixedPointSet->SetPointData(pPoints);

  typedef itk::PointSetToImageRegistrationMethod<PointSetType, DistanceImageType> RegistrationFilterType;
  RegistrationFilterType::Pointer pRegistration = RegistrationFilterType::New();
  pRegistration->SetInitialTransformParameters(pTransform->GetParameters());
  pRegistration->SetMetric(pMetric);
  pRegistration->SetInterpolator(pInterpolator);
  pRegistration->SetOptimizer(pOptimizer);
  pRegistration->SetTransform(pTransform);
  pRegistration->SetFixedPointSet(pFixedPointSet);
  pRegistration->SetMovingImage(pDistanceImage);

  pCOSilencer->disableOutput();
  pRegistration->Update();
  pCOSilencer->enableOutput();


  typedef itk::TransformMeshFilter<DataType, DataType, TransformType> TransformMeshFilterType;
  TransformMeshFilterType::Pointer pTransformMeshFilter = TransformMeshFilterType::New();
  pTransformMeshFilter->SetInput(pModel->GetRepresenter()->GetReference());
  pTransformMeshFilter->SetTransform(pTransform);
  pTransformMeshFilter->Update();

  DataType::Pointer pFittedMesh = pTransformMeshFilter->GetOutput();
  if (opt.strOutputFittedMeshFileName != "")
  {
    saveMesh(pFittedMesh, opt.strOutputFittedMeshFileName);
  }
  if (opt.strOutputProjectedMeshFileName != "")
  {
    DataType::Pointer pProjectedMesh = projectOnTargetMesh<PointsLocatorType>(pFittedMesh, pTargetMesh);
    saveMesh(pProjectedMesh, opt.strOutputProjectedMeshFileName);
  }
}
