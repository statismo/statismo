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
 * This example shows how the fitting of a statistical deformation model can be performed with statismo.
 */

#include <itkCommand.h>
#include <itkImageFileReader.h>
#include <itkImageRegistrationMethod.h>
#include <itkLBFGSOptimizer.h>
#include <itkLinearInterpolateImageFunction.h>
#include <itkMeanSquaresImageToImageMetric.h>
#include <itkNormalizedCorrelationImageToImageMetric.h>

#include "itkInterpolatingStatisticalDeformationModelTransform.h"
#include "itkStandardImageRepresenter.h"
#include "itkStatisticalModel.h"
#include "itkStatismoIO.h"

const unsigned Dimensions = 2;
typedef itk::Image<unsigned short int, Dimensions> ImageType;
typedef itk::Image< itk::Vector<float, ImageType::ImageDimension> , ImageType::ImageDimension > VectorImageType;

typedef itk::StandardImageRepresenter<itk::Vector<float, Dimensions>, Dimensions> RepresenterType;

typedef itk::ImageFileReader<ImageType> ImageReaderType;
//typedef itk::MeanSquaresImageToImageMetric<ImageType, ImageType> MetricType;
typedef itk::NormalizedCorrelationImageToImageMetric<ImageType, ImageType> MetricType;
typedef itk::InterpolatingStatisticalDeformationModelTransform<VectorImageType, double, Dimensions> TransformType;
typedef itk::LinearInterpolateImageFunction<ImageType, double> InterpolatorType;
typedef itk::ImageRegistrationMethod<ImageType, ImageType> RegistrationFilterType;

typedef  itk::LBFGSOptimizer OptimizerType;


typedef itk::StatisticalModel<VectorImageType> StatisticalModelType;



class IterationStatusObserver : public itk::Command {
  public:
    typedef  IterationStatusObserver   Self;
    typedef  itk::Command             Superclass;
    typedef  itk::SmartPointer<Self>  Pointer;

    itkNewMacro( Self );

    typedef itk::LBFGSOptimizer    OptimizerType;

    typedef const OptimizerType                     *OptimizerPointer;


    void Execute(itk::Object *caller, const itk::EventObject & event) {
        Execute( (const itk::Object *)caller, event);
    }

    void Execute(const itk::Object * object, const itk::EventObject & event) {
        OptimizerPointer optimizer =
            dynamic_cast< OptimizerPointer >( object );

        if( ! itk::IterationEvent().CheckEvent( &event ) ) {
            return;
        }

        std::cout << "Iteration: " << ++m_iter_no ;
        std::cout << "; Value: " << optimizer->GetCachedValue();
        std::cout << "; Current Parameters: " << optimizer->GetCachedCurrentPosition() << std::endl;
    }


  protected:
    IterationStatusObserver():
        m_iter_no(0)     {};

    virtual ~IterationStatusObserver() {};

  private:
    int m_iter_no;

};

/*
 * The fixedImage needs to correspond to the image that was used to obtain the displacement fields of the model
 * (e.g. the fixed image in the registration that generated the displacement fields).
 */
int main(int argc, char* argv[]) {

    if (argc < 5) {
        std::cout << "usage " << argv[0] << " modelname fixedImage movingImage output-df" << std::endl;
        exit(-1);
    }

    char* modelname = argv[1];
    char* referencename = argv[2];
    char* targetname = argv[3];
    char* outdfname = argv[4];




    ImageReaderType::Pointer refReader = ImageReaderType::New();
    refReader->SetFileName(referencename);
    refReader->Update();
    ImageType::Pointer refImage = refReader->GetOutput();

    ImageReaderType::Pointer targetReader = ImageReaderType::New();
    targetReader->SetFileName(targetname);
    targetReader->Update();
    ImageType::Pointer targetImage = targetReader->GetOutput();

    RepresenterType::Pointer representer = RepresenterType::New();
    StatisticalModelType::Pointer model = StatisticalModelType::New();
    model = itk::StatismoIO<VectorImageType>::LoadStatisticalModel(representer, modelname);

    // do the fitting
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
    registration->SetFixedImage( refImage );
    registration->SetFixedImageRegion(refImage->GetBufferedRegion() ); // seems to be necessary for the filter to work
    registration->SetMovingImage( targetImage );

    try {

        registration->Update();

    } catch ( itk::ExceptionObject& o ) {
        std::cout << "caught exception " << o << std::endl;
    }

    VectorImageType::Pointer df = model->DrawSample(transform->GetCoefficients());

    itk::ImageFileWriter<VectorImageType>::Pointer writer = itk::ImageFileWriter<VectorImageType>::New();
    writer->SetFileName(outdfname);
    writer->SetInput(df);
    writer->Update();

}

