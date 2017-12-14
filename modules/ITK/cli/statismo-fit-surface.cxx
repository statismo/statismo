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

#include <itkBoundingBox.h>
#include <itkCompositeTransform.h>
#include <itkSignedMaurerDistanceMapImageFilter.h>
#include <itkLBFGSOptimizer.h>
#include <itkLinearInterpolateImageFunction.h>
#include <itkMesh.h>
#include <itkPointSetToImageFilter.h>
#include <itkPointSetToImageRegistrationMethod.h>
#include <itkPointsLocator.h>
#include <itkReducedVarianceModelBuilder.h>
#include <itkStandardMeshRepresenter.h>
#include <itkStatismoIO.h>
#include <itkStatisticalModel.h>
#include <itkStatisticalShapeModelTransform.h>
#include <itkVersorRigid3DTransform.h>

#include "utils/itkPenalizingMeanSquaresPointSetToImageMetric.h"
#include "utils/statismo-build-models-utils.h"
#include "utils/statismo-fitting-utils.h"

namespace po = boost::program_options;
using namespace std;

const unsigned Dimensions = 3;
typedef itk::Mesh<float, Dimensions> DataType;
typedef itk::Image<float, Dimensions> DistanceImageType;
typedef itk::StatisticalModel<DataType> StatisticalModelType;




struct programOptions {
    bool bDisplayHelp;
    bool bPrintFittingInformation;

    string strInputModelFileName;
    string strInputTargetMeshFileName;

    string strOutputFittedMeshFileName;
    string strOutputProjectedMeshFileName;

    double dRegularizationWeight;
    unsigned uNumberOfIterations;
    string strInputFixedLandmarksFileName;
    string strInputMovingLandmarksFileName;
    double dLandmarksVariance;
};

po::options_description initializeProgramOptions(programOptions& poParameters);
bool isOptionsConflictPresent(programOptions& opt);
void fitMesh(programOptions opt, ConsoleOutputSilencer* pCOSilencer);



int main(int argc, char** argv) {
    programOptions poParameters;
    po::options_description optAllOptions = initializeProgramOptions(poParameters);


    po::variables_map vm;
    try {
        po::parsed_options parsedOptions = po::command_line_parser(argc, argv).options(optAllOptions).run();
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
        fitMesh(poParameters, &coSilencer);
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

    //at least one of the outputs needs to be set or the fitting information has to be printed
    if (opt.strOutputFittedMeshFileName == "" && opt.strOutputProjectedMeshFileName == "") {
        return true;
    }

    if (opt.strInputModelFileName == "" || opt.strInputTargetMeshFileName == "") {
        return true;
    }

    return false;
}



DistanceImageType::Pointer computeDistanceImageForMesh(DataType::Pointer pMesh, const unsigned uDistImageResolution = 256) {
    // Compute a bounding box around the reference shape
    typedef  itk::BoundingBox<int, Dimensions, float, DataType::PointsContainer> BoundingBoxType;
    BoundingBoxType::Pointer pBoundingBox = BoundingBoxType::New();
    pBoundingBox->SetPoints(pMesh->GetPoints());
    pBoundingBox->ComputeBoundingBox();

    // Compute a binary image from the point set, which is as large as the bounding box plus a margin.
    typedef itk::Image< unsigned char, Dimensions > BinaryImageType;
    typedef itk::PointSet<float, Dimensions  > PointSetType;
    typedef itk::PointSetToImageFilter<PointSetType, BinaryImageType> PointsToImageFilterType;
    PointsToImageFilterType::Pointer pointsToImageFilter = PointsToImageFilterType::New();
    pointsToImageFilter->SetInput(pMesh);
    BinaryImageType::SpacingType spacing;
    BinaryImageType::SpacingType margin;
    BinaryImageType::PointType origin = pBoundingBox->GetMinimum();
    BinaryImageType::SpacingType diff = pBoundingBox->GetMaximum() - pBoundingBox->GetMinimum();
    BinaryImageType::SizeType size;

    for (unsigned i = 0; i < Dimensions; ++i) {
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

void saveMesh(DataType::Pointer pData, const string& strFileName) {
    typedef itk::MeshFileWriter<DataType> DataWriterType;
    DataWriterType::Pointer pDataWriter = DataWriterType::New();
    pDataWriter->SetFileName(strFileName);
    pDataWriter->SetInput(pData);
    pDataWriter->Update();
}

template<class PointsLocatorType>
DataType::Pointer projectOnTargetMesh(DataType::Pointer pMesh, DataType::Pointer pTargetMesh) {
    typename PointsLocatorType::Pointer ptLocator = PointsLocatorType::New();
    ptLocator->SetPoints(pTargetMesh->GetPoints());
    ptLocator->Initialize();
    DataType::Pointer projectedMesh = cloneMesh<DataType>(pMesh);
    for (DataType::PointsContainer::Iterator pointIter = projectedMesh->GetPoints()->Begin(); pointIter != projectedMesh->GetPoints()->End(); ++pointIter) {
        unsigned uClosestPointId = ptLocator->FindClosestPoint(pointIter->Value());
        pointIter->Value() = pTargetMesh->GetPoint(uClosestPointId);
    }
    return projectedMesh;
}


void fitMesh(programOptions opt, ConsoleOutputSilencer* pCOSilencer) {
    typedef itk::MeshFileReader<DataType> MeshReaderType;
    MeshReaderType::Pointer pMeshReader = MeshReaderType::New();
    pMeshReader->SetFileName(opt.strInputTargetMeshFileName.c_str());
    pMeshReader->Update();
    DataType::Pointer pTargetMesh = pMeshReader->GetOutput();

    typedef itk::StandardMeshRepresenter<float, Dimensions> RepresenterType;
    RepresenterType::Pointer pRepresenter = RepresenterType::New();
    StatisticalModelType::Pointer pModel = StatisticalModelType::New();
    pModel = itk::StatismoIO<DataType>::LoadStatisticalModel(pRepresenter.GetPointer(),
                                                             opt.strInputModelFileName.c_str());

    StatisticalModelType::Pointer pConstrainedModel;
    typedef itk::StatisticalShapeModelTransform<DataType, double, Dimensions> StatisticalModelTransformType;
    typedef itk::Transform<double, Dimensions, Dimensions> TransformType;
    TransformType::Pointer pTransform;

    typedef itk::PointsLocator< DataType::PointsContainer > PointsLocatorType;
    if (opt.strInputFixedLandmarksFileName == "") {
        typedef  itk::BoundingBox<int, Dimensions, float, DataType::PointsContainer> BoundingBoxType;
        //Compute bounding box of model mesh
        BoundingBoxType::Pointer pModelMeshBox = BoundingBoxType::New();
        pModelMeshBox->SetPoints(pModel->GetRepresenter()->GetReference()->GetPoints());
        pModelMeshBox->ComputeBoundingBox();

        //Compute bounding box of target mesh
        BoundingBoxType::Pointer pTargetMeshBox = BoundingBoxType::New();
        pTargetMeshBox->SetPoints(pTargetMesh->GetPoints());
        pTargetMeshBox->ComputeBoundingBox();

        pConstrainedModel = pModel;
        StatisticalModelTransformType::Pointer pModelTransform = StatisticalModelTransformType::New();
        pModelTransform->SetStatisticalModel(pModel);
        pModelTransform->SetIdentity();

        //No Landmarks are available: we also have to allow rotation and translation.
        typedef itk::VersorRigid3DTransform<double> RotationAndTranslationTransformType;
        RotationAndTranslationTransformType::Pointer pRotationAndTranslationTransform = RotationAndTranslationTransformType::New();
        pRotationAndTranslationTransform->SetIdentity();

        RotationAndTranslationTransformType::CenterType center = pModelMeshBox->GetCenter();
        pRotationAndTranslationTransform->SetCenter(center);

        RotationAndTranslationTransformType::TranslationType translation = pTargetMeshBox->GetCenter() - pModelMeshBox->GetCenter();
        pRotationAndTranslationTransform->SetTranslation(translation);

        typedef itk::CompositeTransform<double, Dimensions> CompositeTransformType;
        CompositeTransformType::Pointer pCompositeTransform = CompositeTransformType::New();
        pCompositeTransform->AddTransform(pRotationAndTranslationTransform);
        pCompositeTransform->AddTransform(pModelTransform);
        pCompositeTransform->SetAllTransformsToOptimizeOn();

        pTransform = pCompositeTransform;
    } else {
        pConstrainedModel = buildPosteriorShapeModel<DataType, StatisticalModelType, PointsLocatorType>(pModel, opt.strInputFixedLandmarksFileName, opt.strInputMovingLandmarksFileName, opt.dLandmarksVariance);
        StatisticalModelTransformType::Pointer pModelTransform = StatisticalModelTransformType::New();
        pModelTransform->SetStatisticalModel(pConstrainedModel);
        pModelTransform->SetIdentity();
        pTransform = pModelTransform;
    }


    DistanceImageType::Pointer pDistanceImage = computeDistanceImageForMesh(pTargetMesh);

    typedef itk::LBFGSOptimizer OptimizerType;
    OptimizerType::Pointer pOptimizer = OptimizerType::New();
    initializeOptimizer<OptimizerType>(pOptimizer, opt.uNumberOfIterations, pModel->GetNumberOfPrincipalComponents(), pTransform->GetNumberOfParameters(), opt.bPrintFittingInformation, pCOSilencer);

    typedef itk::LinearInterpolateImageFunction<DistanceImageType, double> InterpolatorType;
    InterpolatorType::Pointer pInterpolator = InterpolatorType::New();

    typedef itk::PointSet<float, Dimensions > PointSetType;
    typedef itk::PenalizingMeanSquaresPointSetToImageMetric<PointSetType, DistanceImageType> MetricType;
    MetricType::Pointer pMetric = MetricType::New();
    pMetric->SetRegularizationParameter(opt.dRegularizationWeight);
    pMetric->SetNumberOfModelComponents(pConstrainedModel->GetNumberOfPrincipalComponents());

    DataType::Pointer pReference= pModel->GetRepresenter()->GetReference();
    PointSetType::Pointer pFixedPointSet = PointSetType::New();
    pFixedPointSet->SetPoints(pReference->GetPoints());
    PointSetType::PointDataContainer::Pointer pPoints = PointSetType::PointDataContainer::New();
    pPoints->Reserve(pReference->GetNumberOfPoints());
    for (PointSetType::PointDataContainer::Iterator it = pPoints->Begin(); it != pPoints->End(); ++it) {
        it->Value() = 0;
    }
    pFixedPointSet->SetPointData(pPoints);

    typedef itk::PointSetToImageRegistrationMethod<PointSetType,DistanceImageType > RegistrationFilterType;
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
    if(opt.strOutputFittedMeshFileName != "") {
        saveMesh(pFittedMesh, opt.strOutputFittedMeshFileName);
    }
    if(opt.strOutputProjectedMeshFileName != "") {
        DataType::Pointer pProjectedMesh = projectOnTargetMesh<PointsLocatorType>(pFittedMesh, pTargetMesh);
        saveMesh(pProjectedMesh, opt.strOutputProjectedMeshFileName);
    }
}


po::options_description initializeProgramOptions(programOptions& poParameters) {
    po::options_description optMandatory("Mandatory options");
    optMandatory.add_options()

    ("input-model,i", po::value<string>(&poParameters.strInputModelFileName), "The path to the model file.")
    ("input-targetmesh,t", po::value<string>(&poParameters.strInputTargetMeshFileName), "The path to the target mesh.")
    ("number-of-iterations,n", po::value<unsigned>(&poParameters.uNumberOfIterations)->default_value(100), "Number of iterations")
    ("regularization-weight,w", po::value<double>(&poParameters.dRegularizationWeight), "This is the regularization weight to make sure the model parameters don't don't get too big while fitting.")
    ;

    po::options_description optOutput("Mandatory Output options (set at least one)");
    optOutput.add_options()
    ("output-fit,o", po::value<string>(&poParameters.strOutputFittedMeshFileName), "Name of the output file where the fitted mesh will be written to.")
    ("output-projected,j", po::value<string>(&poParameters.strOutputProjectedMeshFileName), "Name of the output file where the projected mesh will be written to.")
    ;

    po::options_description optLandmarks("Landmarks (optional - if you set one you have to set all)");
    optLandmarks.add_options()
    ("fixed-landmarks,f", po::value<string>(&poParameters.strInputFixedLandmarksFileName), "Name of the file where the fixed Landmarks are saved.")
    ("moving-landmarks,m", po::value<string>(&poParameters.strInputMovingLandmarksFileName), "Name of the file where the moving Landmarks are saved.")
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
