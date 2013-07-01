/*
 * This file is part of the statismo library.
 *
 * Author: Marcel Luethi (marcel.luethi@unibas.ch)
 *         Ghazi Bouabene (ghazi.bouabene@unibas.ch)
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

/**
 * This code serves as an example, how shape model fitting can be done using Statismo.
 * The method uses the landmark pairs to estimate the pose (rigid transform) of the image.
 * The fixed landmarks are points of the reference that was used to build the shape model
 * and the moving landmarks are the corresponding points on the target image.
 * To actually fit the new model to the image, we use the itk PointSetToImageRegistration method.
 * The fitting is not performed on the original image, but a feature image, which is obtained by performing a threshold
 * segmentation, followed by a distance transform.
 *
 */



#include "itkMeshRepresenter.h"
#include "statismo_ITK/itkStatisticalModel.h"
#include "statismo_ITK/itkStatisticalShapeModelTransform.h"
#include "itkMeanSquaresPointSetToImageMetric.h"
#include "itkLinearInterpolateImageFunction.h"
#include "itkLBFGSOptimizer.h"
#include "itkPointSetToImageRegistrationMethod.h"
#include "itkMeshFileReader.h"
#include "itkMeshFileWriter.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkCommand.h"
#include "itkMesh.h"
#include "itkTransformMeshFilter.h"
#include "itkCompositeTransform.h"
#include "itkRigid3DTransform.h"
#include "itkLandmarkBasedTransformInitializer.h"
#include "itkCastImageFilter.h"
#include "itkCannyEdgeDetectionImageFilter.h"
#include "itkSignedDanielssonDistanceMapImageFilter.h"
#include "itkBinaryThresholdImageFilter.h"
#include "itkCompositeTransform.h"
#include "itkPartiallyFixedModelBuilder.h"
#include "itkPointsLocator.h"

const unsigned Dimensions = 3;
typedef itk::Mesh<float, Dimensions  > MeshType;
typedef itk::Point<double, 3> PointType;
typedef itk::Image<float, Dimensions> CTImageType;
typedef itk::Image<float, Dimensions> DistanceImageType;
typedef itk::ImageFileReader<CTImageType> CTImageReaderType;
typedef itk::ImageFileWriter<DistanceImageType> DistanceImageWriterType;
typedef itk::MeshRepresenter<float, Dimensions> RepresenterType;
typedef itk::MeshFileReader<MeshType> MeshReaderType;
typedef itk::Image< itk::CovariantVector<float, Dimensions>, Dimensions >   GradientImageType;
typedef itk::MeanSquaresPointSetToImageMetric<MeshType, DistanceImageType> MetricType;
typedef itk::StatisticalShapeModelTransform<RepresenterType, double, Dimensions> StatisticalModelTransformType;
typedef itk::StatisticalModel<RepresenterType> StatisticalModelType;
typedef itk::PointSetToImageRegistrationMethod<MeshType, DistanceImageType> RegistrationFilterType;
typedef  itk::LBFGSOptimizer OptimizerType;
typedef itk::LinearInterpolateImageFunction<DistanceImageType, double> InterpolatorType;
typedef itk::BinaryThresholdImageFilter <CTImageType, CTImageType>  BinaryThresholdImageFilterType;
typedef itk::SignedDanielssonDistanceMapImageFilter<CTImageType, DistanceImageType> DistanceMapImageFilterType;
typedef itk::VersorRigid3DTransform<double> RigidTransformType;
typedef itk::LandmarkBasedTransformInitializer<RigidTransformType, DistanceImageType, DistanceImageType> LandmarkTransformInitializerType;
typedef itk::CompositeTransform<double, 3> CompositeTransformType;
typedef itk::TransformMeshFilter<MeshType, MeshType, CompositeTransformType> TransformMeshFilterType;


typedef itk::PartiallyFixedModelBuilder<RepresenterType> PartiallyFixedModelBuilderType;
typedef itk::PointsLocator<int, 3, double, MeshType::PointsContainer > PointsLocatorType;

class ConfigParameters {
public:
// Some configuration parameters
static const short maxNumberOfIterations = 10000; // the maximum number of iterations to use in the optimization
static const double translationScale; // dynamic range of translations
static const double rotationScale; // dynamic range of rotations
static const double smScale; // dynamic range of statistical model parameters
};
double const ConfigParameters::translationScale = 1;
double const ConfigParameters::rotationScale = 0.1;
double const ConfigParameters::smScale = 3;


class Utils {
public:
	/**
	 * read landmarks from the given file in slicer fcsv formant and return them as a list.
	 *
	 * The format is: label,x,y,z
	 *
	 * @param filename the filename
	 * @returns A list of itk points
	 */
	static std::vector<PointType > readLandmarks(const std::string& filename) {

		std::vector<PointType> ptList;

		std::fstream file ( filename.c_str() );
		if (!file) {
			std::cout << "could not read landmark file " << std::endl;
			throw std::runtime_error("could not read landmark file ");
		}
		std::string line;
		while (  std::getline ( file, line))
		{
			if (line.length() > 0 && line[0] == '#')
				continue;

			std::istringstream strstr(line);
			std::string token;
			std::getline(strstr, token, ','); // ignore the label
			std::getline(strstr, token, ','); // get the x coord
			double pt0 = atof(token.c_str());
			std::getline(strstr, token, ','); // get the y coord
			double pt1 = atof(token.c_str());
			std::getline(strstr, token, ','); // get the z coord
			double pt2 = atof(token.c_str());
			PointType pt;
			pt[0] = pt0; pt[1] = pt1; pt[2] = pt2;
			ptList.push_back(pt);
		}
		return ptList;
	}

};


// Returns a new model, that is restricted to go through the proints specified in targetLandmarks..
//
StatisticalModelType::Pointer
computePartiallyFixedModel(const RigidTransformType* rigidTransform,
												const StatisticalModelType* statisticalModel,
												const  std::vector<PointType >& modelLandmarks,
												const  std::vector<PointType >& targetLandmarks,
												double variance)
{

		// invert the transformand back transform the landmarks
		RigidTransformType::Pointer rinv = RigidTransformType::New();
		rigidTransform->GetInverse(rinv);

	    StatisticalModelType::PointValueListType constraints;

	    // We need to make sure the the points in fixed landmarks are real vertex points of the model reference.
		MeshType::Pointer reference = statisticalModel->GetRepresenter()->GetReference();
		PointsLocatorType::Pointer ptLocator = PointsLocatorType::New();
		ptLocator->SetPoints(reference->GetPoints());
		ptLocator->Initialize();

		assert(refLandmarks.size() == targetLandmarks.size());
	    for (unsigned i = 0; i < targetLandmarks.size(); i++) {

			int closestPointId = ptLocator->FindClosestPoint(modelLandmarks[i]);
			PointType refPoint = (*reference->GetPoints())[closestPointId];

			// compensate for the rigid transformation that was applied to the model
			PointType targetLmAtModelPos = rinv->TransformPoint(targetLandmarks[i]);
			StatisticalModelType::PointValuePairType pointValue(refPoint ,targetLmAtModelPos);
			constraints.push_back(pointValue);

		}

		PartiallyFixedModelBuilderType::Pointer partiallyFixedModelBuilder = PartiallyFixedModelBuilderType::New();
	    StatisticalModelType::Pointer partiallyFixedModel = partiallyFixedModelBuilder->BuildNewModelFromModel(statisticalModel,constraints, variance, false);

	    return partiallyFixedModel;
}



//
// This class is used to track the progress of the optimization
// (its method Execute is called in each iteration of the optimization)
//
class IterationStatusObserver : public itk::Command
{
public:
  typedef  IterationStatusObserver   Self;
  typedef  itk::Command             Superclass;
  typedef  itk::SmartPointer<Self>  Pointer;

  itkNewMacro( Self );

  typedef itk::LBFGSOptimizer    OptimizerType;
  //typedef itk::GradientDescentOptimizer OptimizerType;

  typedef const OptimizerType                     *OptimizerPointer;


   void Execute(itk::Object *caller, const itk::EventObject & event)
  {
    Execute( (const itk::Object *)caller, event);
  }

  void Execute(const itk::Object * object, const itk::EventObject & event)
  {
    OptimizerPointer optimizer =
                         dynamic_cast< OptimizerPointer >( object );

    if( ! itk::IterationEvent().CheckEvent( &event ) )
    {
      return;
    }

    std::cout << "Iteration: " << ++m_iter_no ;
   std::cout << "; Value: " << optimizer->GetCachedValue();
    std::cout << "; Current Parameters: " << optimizer->GetCachedCurrentPosition() << std::endl;
  }


protected:
  IterationStatusObserver():
     m_iter_no(0)     {};

  virtual ~IterationStatusObserver(){};

private:
  int m_iter_no;

};


/**
 * The input to the program is:
 * -  A shape model (a statismo model that was built using the itk::MeshRepresenter
 * -  A 3D CT image
 * -  Landmarks on the reference used to build the shape model
 * -  Landmarks on the target image
 * -  A threshold that yields a rough segmentation of the structure to be fitted
 * - the name of the output
 *
 */
int main(int argc, char* argv[]) {

	if (argc < 8) {
		std::cout << "usage " << argv[0] << " modelname ctimage fixedLandmarks movingLandmarks threshold lmVariance outputmesh" << std::endl;
		exit(-1);
	}

	char* modelName = argv[1];
	char* targetName = argv[2];
	char* fixedLandmarksName = argv[3];
	char* movingLandmarksName = argv[4];
	short threshold = atoi(argv[5]);
	double lmVariance = atof(argv[6]);
	char* outputMeshName = argv[7];


	// load the image to which we will fit
	CTImageReaderType::Pointer targetReader = CTImageReaderType::New();
	targetReader->SetFileName(targetName);
	targetReader->Update();
	CTImageType::Pointer ctImage = targetReader->GetOutput();


	// We compute a binary threshold of input image to get a rough segmentation of the bony structure.
	// Then we compute a distance transform of the segmentation, which we then use for the fitting
	BinaryThresholdImageFilterType::Pointer thresholdFilter = BinaryThresholdImageFilterType::New();
	thresholdFilter->SetInput(ctImage);
	thresholdFilter->SetLowerThreshold( threshold );
	DistanceMapImageFilterType::Pointer dm = DistanceMapImageFilterType::New();
	dm->SetInput(thresholdFilter->GetOutput());
	dm->Update();
	DistanceImageType::Pointer distanceImage = dm->GetOutput();

	// read the landmarks
	std::vector<PointType> fixedLandmarks = Utils::readLandmarks(fixedLandmarksName);
	std::vector<PointType> movingLandmarks = Utils::readLandmarks(movingLandmarksName);

	// initialize the rigid transform
	RigidTransformType::Pointer rigidTransform = RigidTransformType::New();
	LandmarkTransformInitializerType::Pointer initializer = LandmarkTransformInitializerType::New();
	initializer->SetFixedLandmarks(fixedLandmarks);
	initializer->SetMovingLandmarks(movingLandmarks);
	initializer->SetTransform(rigidTransform);
	initializer->InitializeTransform();

	// load the model create a shape model transform with it
	StatisticalModelType::Pointer model = StatisticalModelType::New();
	model->Load(modelName);

	StatisticalModelType::Pointer constraintModel = computePartiallyFixedModel(rigidTransform, model, fixedLandmarks, movingLandmarks, lmVariance);

	StatisticalModelTransformType::Pointer statModelTransform = StatisticalModelTransformType::New();
	statModelTransform->SetStatisticalModel(constraintModel);
	statModelTransform->SetIdentity();

	// compose the two transformation
	CompositeTransformType::Pointer transform = CompositeTransformType::New();
	transform->AddTransform(rigidTransform);
	transform->AddTransform(statModelTransform);
	transform->SetOnlyMostRecentTransformToOptimizeOn(); //  only optimize the shape parameters, not the rigid transform parameters
	//transform->SetAllTransformsToOptimizeOn(); // optimize shape and pose parameters

	// Setting up the fitting
	OptimizerType::Pointer optimizer = OptimizerType::New();
	optimizer->SetMaximumNumberOfFunctionEvaluations(ConfigParameters::maxNumberOfIterations);
	optimizer->MinimizeOn();

	unsigned numStatmodelParameters = statModelTransform->GetNumberOfParameters();
	unsigned totalNumParameters =  rigidTransform->GetNumberOfParameters() + numStatmodelParameters;

	// set the scales of the optimizer, to compensate for potentially different scales of translation, rotation and shape parameters
	OptimizerType::ScalesType scales( totalNumParameters );
	for (unsigned i = 0; i < numStatmodelParameters; i++) {
		scales[i] = 1.0 / (ConfigParameters::smScale);
	}
	for (unsigned i = numStatmodelParameters; i < numStatmodelParameters + 3; i++) {
		scales[i] =  1.0 / (ConfigParameters::rotationScale);
	}
	for (unsigned i = numStatmodelParameters + 3; i < statModelTransform->GetNumberOfParameters() + 6; i++) {
		scales[i] = 1.0 / (ConfigParameters::translationScale);
	}
	optimizer->SetScales(scales);



	// set up the observer to keep track of the progress
	typedef  IterationStatusObserver ObserverType;
	ObserverType::Pointer observer = ObserverType::New();
	optimizer->AddObserver( itk::IterationEvent(), observer );

	// set up the metric and interpolators
	MetricType::Pointer metric = MetricType::New();
	InterpolatorType::Pointer interpolator = InterpolatorType::New();

	// connect all the components
	RegistrationFilterType::Pointer registration = RegistrationFilterType::New();
	registration->SetInitialTransformParameters(transform->GetParameters());
	registration->SetInterpolator(interpolator);
	registration->SetMetric(metric);
	registration->SetOptimizer(   optimizer);
	registration->SetTransform(   transform );


	// the input to the registration will be the reference of the statistical model and the
	// distance map we computed above.
	registration->SetFixedPointSet(  model->GetRepresenter()->GetReference() );
	registration->SetMovingImage(distanceImage);

	try {
		std::cout << "starting model fitting" << std::endl;
		registration->Update();

	} catch ( itk::ExceptionObject& o ) {
		std::cout << "caught exception " << o << std::endl;
	}


	TransformMeshFilterType::Pointer transformMeshFilter = TransformMeshFilterType::New();
	transformMeshFilter->SetInput(model->GetRepresenter()->GetReference());
	transformMeshFilter->SetTransform(transform);

	// Write out the fitting result
	itk::MeshFileWriter<MeshType>::Pointer writer = itk::MeshFileWriter<MeshType>::New();
	writer->SetFileName(outputMeshName);
	writer->SetInput(transformMeshFilter->GetOutput());
	writer->Update();

}



