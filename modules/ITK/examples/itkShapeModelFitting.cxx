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
 * This example shows how the fitting of a statistical shape model to a Mesh can be performed with Statismo and the itk
 * Registration framework.
 */
#include <itkCommand.h>
#include <itkEuclideanDistancePointMetric.h>
#include <itkLevenbergMarquardtOptimizer.h>
#include <itkMesh.h>
#include <itkMeshFileReader.h>
#include <itkPointSetToPointSetRegistrationMethod.h>

#include "statismo/ITK/itkStandardMeshRepresenter.h"
#include "statismo/ITK/itkIO.h"
#include "statismo/ITK/itkStatisticalModel.h"
#include "statismo/ITK/itkStatisticalShapeModelTransform.h"


const unsigned                       Dimensions = 3;
typedef itk::Mesh<float, Dimensions> MeshType;

typedef itk::StandardMeshRepresenter<float, Dimensions> RepresenterType;

typedef itk::MeshFileReader<MeshType>                         MeshReaderType;
typedef itk::EuclideanDistancePointMetric<MeshType, MeshType> MetricType;

// As a transform, we use the StatisticalShapeModelTransform, that comes with statismo
typedef itk::StatisticalShapeModelTransform<MeshType, double, Dimensions> TransformType;


typedef itk::PointSetToPointSetRegistrationMethod<MeshType, MeshType> RegistrationFilterType;

typedef itk::LevenbergMarquardtOptimizer OptimizerType;

typedef itk::StatisticalModel<MeshType> StatisticalModelType;


class IterationStatusObserver : public itk::Command
{
public:
  typedef IterationStatusObserver Self;
  typedef itk::Command            Superclass;
  typedef itk::SmartPointer<Self> Pointer;

  itkNewMacro(Self);


  typedef const OptimizerType * OptimizerPointer;


  void
  Execute(itk::Object * caller, const itk::EventObject & event)
  {
    Execute((const itk::Object *)caller, event);
  }

  void
  Execute(const itk::Object * object, const itk::EventObject & event)
  {
    OptimizerPointer optimizer = dynamic_cast<OptimizerPointer>(object);

    if (!itk::IterationEvent().CheckEvent(&event))
    {
      return;
    }

    std::cout << "Iteration: " << ++m_iter_no << " model arameters " << optimizer->GetCachedCurrentPosition()
              << std::endl;
  }


protected:
  IterationStatusObserver()
    : m_iter_no(0){};

  virtual ~IterationStatusObserver(){};

private:
  int m_iter_no;
};


int
main(int argc, char * argv[])
{

  if (argc < 4)
  {
    std::cout << "usage " << argv[0] << " modelname target outputmesh" << std::endl;
    exit(-1);
  }

  char * modelname = argv[1];
  char * targetname = argv[2];
  char * outputmeshname = argv[3];

  // load the model
  RepresenterType::Pointer      representer = RepresenterType::New();
  StatisticalModelType::Pointer model = StatisticalModelType::New();
  model = itk::StatismoIO<MeshType>::LoadStatisticalModel(representer, modelname);
  MeshType::Pointer fixedPointSet = model->GetRepresenter()->GetReference();
  std::cout << "model succesully loaded " << std::endl;

  // load the image to which we will fit
  MeshReaderType::Pointer targetReader = MeshReaderType::New();
  targetReader->SetFileName(targetname);
  targetReader->Update();
  MeshType::Pointer targetMesh = targetReader->GetOutput();


  // now we perform the fitting, using the itk registration framework
  TransformType::Pointer transform = TransformType::New();
  transform->SetStatisticalModel(model);
  transform->SetIdentity();

  // Setting up the fitting
  OptimizerType::Pointer optimizer = OptimizerType::New();
  optimizer->SetNumberOfIterations(100);
  optimizer->SetUseCostFunctionGradient(false);
  optimizer->SetGradientTolerance(1e-5);
  optimizer->SetValueTolerance(1e-5);
  optimizer->SetEpsilonFunction(1e-6);


  typedef IterationStatusObserver ObserverType;
  ObserverType::Pointer           observer = ObserverType::New();
  optimizer->AddObserver(itk::IterationEvent(), observer);

  MetricType::Pointer metric = MetricType::New();


  RegistrationFilterType::Pointer registration = RegistrationFilterType::New();
  registration->SetInitialTransformParameters(transform->GetParameters());
  registration->SetMetric(metric);
  registration->SetOptimizer(optimizer);
  registration->SetTransform(transform);
  registration->SetFixedPointSet(targetMesh);
  registration->SetMovingPointSet(fixedPointSet);


  try
  {
    std::cout << "starting model fitting" << std::endl;
    registration->Update();
  }
  catch (itk::ExceptionObject & o)
  {
    std::cout << "caught exception " << o << std::endl;
  }

  // We obtain the fitting result by drawing the model instance that belongs to the
  // optimal tranform parameters (coefficients)
  MeshType::Pointer mesh = model->DrawSample(transform->GetCoefficients());

  // Write out the fitting result
  itk::MeshFileWriter<MeshType>::Pointer writer = itk::MeshFileWriter<MeshType>::New();
  writer->SetFileName(outputmeshname);
  writer->SetInput(mesh);
  writer->Update();
}
