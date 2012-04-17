/*
 * This file is part of the statismo library.
 *
 * Author: Marcel Luethi (marcel.luethi@unibas.ch)
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

/*
 * This example shows how the fitting of a statistical shape model can be performed with statismo.
 */

#include "itkMeshRepresenter.h"
#include "itkStatisticalModel.h"
#include "itkStatisticalShapeModelTransform.h"
#include "itkMeanSquaresPointSetToImageMetric.h"
#include "itkLBFGSOptimizer.h"
#include "itkPointSetToImageRegistrationMethod.h"
#include "itkImageFileReader.h"
#include "itkCommand.h"
#include "itkLinearInterpolateImageFunction.h"
#include "itkMesh.h"


const unsigned Dimensions = 3;
typedef itk::Image<float, Dimensions> ImageType;
typedef itk::Mesh<float, Dimensions  > MeshType;

typedef itk::MeshRepresenter<float, Dimensions> RepresenterType;

typedef itk::ImageFileReader<ImageType> ImageReaderType;
typedef itk::MeanSquaresPointSetToImageMetric<MeshType, ImageType> MetricType;

// As a transform, we use the StatisticalShapeModelTransform, that comes with statismo
typedef itk::StatisticalShapeModelTransform<RepresenterType, double, Dimensions> TransformType;


typedef itk::PointSetToImageRegistrationMethod<MeshType, ImageType> RegistrationFilterType;
typedef itk::LinearInterpolateImageFunction<ImageType, double> InterpolatorType;

typedef  itk::LBFGSOptimizer OptimizerType;

typedef itk::StatisticalModel<RepresenterType> StatisticalModelType;



class IterationStatusObserver : public itk::Command
{
public:
  typedef  IterationStatusObserver   Self;
  typedef  itk::Command             Superclass;
  typedef  itk::SmartPointer<Self>  Pointer;

  itkNewMacro( Self );

  typedef itk::LBFGSOptimizer    OptimizerType;

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


int main(int argc, char* argv[]) {

	if (argc < 4) {
		std::cout << "usage " << argv[0] << " modelname target outputmesh" << std::endl;
		exit(-1);
	}

	char* modelname = argv[1];
	char* targetname = argv[2];
	char* outputmeshname = argv[3];

	// load the model
	StatisticalModelType::Pointer model = StatisticalModelType::New();
	model->Load(modelname);
	MeshType::Pointer fixedPointSet  = model->GetRepresenter()->GetReference();
	std::cout << "model succesully loaded " << std::endl;

	// load the image to which we will fit
	ImageReaderType::Pointer targetReader = ImageReaderType::New();
	targetReader->SetFileName(targetname);
	targetReader->Update();
	ImageType::Pointer targetImage = targetReader->GetOutput();


	// now we perform the fitting, using the itk registration framework
	TransformType::Pointer transform = TransformType::New();
	transform->SetStatisticalModel(model);
	transform->SetIdentity();

	// Setting up the fitting
	OptimizerType::Pointer optimizer = OptimizerType::New();
	optimizer->MinimizeOn();
	optimizer->SetMaximumNumberOfFunctionEvaluations(100);

	typedef  IterationStatusObserver ObserverType;
	ObserverType::Pointer observer = ObserverType::New();
	optimizer->AddObserver( itk::IterationEvent(), observer );

	MetricType::Pointer metric = MetricType::New();
	InterpolatorType::Pointer interpolator = InterpolatorType::New();


	RegistrationFilterType::Pointer registration = RegistrationFilterType::New();
	registration->SetInitialTransformParameters(transform->GetParameters());
	registration->SetMetric(metric);
	registration->SetOptimizer(   optimizer   );
	registration->SetTransform(   transform );
	registration->SetInterpolator( interpolator );
	registration->SetFixedPointSet( fixedPointSet );
	registration->SetMovingImage( targetImage );


	try {
		std::cout << "starting model fitting" << std::endl;
		registration->Update();

	} catch ( itk::ExceptionObject& o ) {
		std::cout << "caught exception " << o << std::endl;
	}

	// We obtain the fitting result by drawing the model instance that belongs to the
	// optimal tranform parameters (coefficients)
	MeshType::Pointer mesh = model->DrawInstance(transform->GetCoefficients());

	// Write out the fitting result
	itk::MeshFileWriter<MeshType>::Pointer writer = itk::MeshFileWriter<MeshType>::New();
	writer->SetFileName(outputmeshname);
	writer->SetInput(mesh);
	writer->Update();

}

